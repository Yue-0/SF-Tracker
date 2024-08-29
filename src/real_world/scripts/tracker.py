from os import path

import tf
import cv2
import rospy
import torch
import numpy as np
from cv_bridge import CvBridge
from onnxruntime import InferenceSession as OnnxModel

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

__author__ = "YueLin"

WAIT, SELECT, TRACKING = range(3)
ONNX = path.join(path.split(path.split(__file__)[0])[0], "onnx")

MEAN, STD = [0.485, 0.456, 0.406], [0.229, 0.224, 0.225]
K = np.array([[386.707, 0, 322.324], [0, 385.641, 234.589], [0, 0, 1]])


class HiT:
    def __init__(self,
                 model: str,
                 device: str = None,
                 margin: int = 10,
                 size: tuple = (128, 256)):
        self.box = None
        self.size = size
        self.margin = margin
        self.template = None
        if device is None:
            device = "CUDA" if torch.cuda.is_available() else "CPU"
        self.model = OnnxModel(
            path.join(ONNX, "{}.onnx".format(model)),
            providers=["{}ExecutionProvider".format(device.upper())]
        )
        self.std = torch.tensor(STD).view((1, 3, 1, 1))
        self.mean = torch.tensor(MEAN).view((1, 3, 1, 1))
    
    def __call__(self, image: np.ndarray) -> list:
        search, resize = self.preprocess(image, True)
        self.post_process(self.inference(search), resize, image.shape[:2])
        return self.box
    
    def init(self, image: np.ndarray, box: list) -> None:
        self.box = box
        self.template = self.preprocess(image, False)[0]
    
    def sample(self, image: np.ndarray, mode: int) -> tuple:
        x, y, w, h = self.box
        height, width, _ = image.shape
        crop = np.ceil(np.sqrt(w * h) * (mode + 1) * 2)
        x1, y1 = round(x + (w - crop) / 2), round(y + (h - crop) / 2)
        x2, y2 = round(x + (w + crop) / 2), round(y + (h + crop) / 2)
        x1, x2, y1, y2 = map(int, (x1, x2, y1, y2))
        left, right, up, down = map(lambda z: max(0, z), (
            -x1, x2 - width + 1, -y1, y2 - height + 1
        ))
        return torch.tensor(cv2.resize(
            cv2.copyMakeBorder(image[
                y1 + up:y2 - down, x1 + left:x2 - right, :
            ], up, down, left, right, cv2.BORDER_CONSTANT),
            (self.size[mode],) * 2
        ) / 255.), self.size[mode] / crop
    
    def inference(self, image: np.ndarray) -> np.ndarray:
        return self.model.run(None, {
            "search": image, "template": self.template
        })[0]

    def preprocess(self, image: np.ndarray, predict: bool) -> tuple:
        image, resize = self.sample(image, int(predict))
        image = image.permute((2, 0, 1)).unsqueeze(0)
        with torch.no_grad():
            image = (image - self.mean) / self.std
        return image.detach().numpy().astype(np.float32), resize

    def post_process(self, box: np.ndarray, resize: float, size: tuple) -> None:
        half = self.size[1] / (resize * 2)
        x, y, w, h = box.reshape((-1, 4)).mean(0) * self.size[1] / resize
        x0, y0 = self.box[0] + self.box[2] / 2, self.box[1] + self.box[3] / 2
        x1, y1 = x + x0 - half - w / 2, y + y0 - half - h / 2
        x2, y2, (h, w) = x1 + w, y1 + h, size
        x1 = min(max(0, x1), w - self.margin)
        y1 = min(max(0, y1), h - self.margin)
        x2 = min(max(self.margin, x2), w)
        y2 = min(max(self.margin, y2), h)
        self.box = list(map(
            lambda b: int(round(b)), [
                x1, y1,
                max(self.margin, x2 - x1),
                max(self.margin, y2 - y1)
            ]
        ))


class Tracker:
    def __init__(self, node: str, window: str = "Tracker"):
        self.mode = WAIT
        self.rect = None
        self.color = None
        self.depth = None
        self.window = window
        rospy.init_node(node)
        self.model = HiT("HiT")
        self.bridge = CvBridge()
        self.result = PoseStamped()
        self._1 = rospy.Subscriber(
            "/frame/color", Image,
            lambda image: self.msg2img(image, True)
        )
        self._2 = rospy.Subscriber(
            "/frame/depth", Image,
            lambda image: self.msg2img(image, False)
        )
        self.listener = tf.TransformListener()
        self.odom_frame = rospy.get_param("odom_frame", "odom")
        self.robot_frame = rospy.get_param("robot_frame", "base_link")
        for topic in ("color", "depth"):
            rospy.wait_for_message("/frame/{}".format(topic), Image)
        self.show = rospy.Publisher(
            "/frame/frame", Image, queue_size=1
        )
        self.publisher = rospy.Publisher(
            "/target", PoseStamped, queue_size=1
        )
        cv2.namedWindow(window)
        self.sleep = rospy.Rate(60).sleep
        self.intrinsics = np.linalg.inv(K)
        self.result.pose.orientation.w = 1
        cv2.setMouseCallback(window, self.mouse)
        self.result.header.frame_id = self.odom_frame
        self.listener.waitForTransform(
            self.odom_frame, self.robot_frame,
            rospy.Time(), rospy.Duration(10.0)
        )
    
    def solve(self, x: int, y: int, w: int, h: int) -> None:
        z = self.depth[
            y - h // 6:y + h // 6, x - w // 6:x + w // 6
        ] * 1e-3
        z = np.mean(z[z != 0])
        sin, cos = np.sin(self.yaw), np.cos(self.yaw)
        x, y, z = z * np.dot(self.intrinsics, np.array([x, y, 1]).T).T
        self.result.pose.position.x = self.x + z * cos + x * sin
        self.result.pose.position.y = self.y + z * sin - x * cos
    
    def mouse(self, event: int, x: int, y: int, *_) -> None:
        if self.mode == SELECT:
            self.rect[2:] = x, y
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.mode == SELECT:
                x0, y0 = self.rect[:2]
                x1, x2 = sorted([x, x0])
                y1, y2 = sorted([y, y0])
                self.model.init(self.color.copy(), [
                    x1, y1, x2 - x1, y2 - y1
                ])
            if self.mode == WAIT:
                self.rect = [x, y, x, y]
            self.mode = (self.mode + 1) % 3
    
    def locate(self) -> None:
        (self.x, self.y, _), angle = self.listener.lookupTransform(
            self.odom_frame, self.robot_frame, rospy.Time()
        )
        self.yaw = tf.transformations.euler_from_quaternion(angle)[-1]
    
    def msg2img(self, image: Image, color: bool) -> None:
        if color:
            self.color = self.bridge.imgmsg_to_cv2(image, "bgr8")
        else:
            self.depth = self.bridge.imgmsg_to_cv2(image, "mono16")

    def tracking(self):
        while not rospy.is_shutdown():
            self.locate()
            frame = self.color.copy()
            if self.mode == SELECT:
                x1, y1, x2, y2 = self.rect
                cv2.rectangle(
                    frame, (min(x1, x2), min(y1, y2)),
                    (max(x1, x2), max(y1, y2)), (0, 0xFF, 0), 2
                )
            if self.mode >= TRACKING:
                x1, y1, w, h = self.model(frame)
                cv2.rectangle(
                    frame, (x1, y1), (x1 + w, y1 + h), (0, 0xFF, 0), 2
                )
                self.solve(x1 + w // 2, y1 + h // 2, w, h)
                self.publisher.publish(self.result)
            self.show.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            cv2.imshow(self.window, frame), cv2.waitKey(1)
            self.sleep()
        cv2.destroyAllWindows()


Tracker("tracker").tracking()
