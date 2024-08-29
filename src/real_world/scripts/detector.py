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

ONNX = path.join(path.split(path.split(__file__)[0])[0], "onnx")

K = np.array([[386.707, 0, 322.324], [0, 385.641, 234.589], [0, 0, 1]])


class YOLOv6:
    def __init__(self, model: str, device: str = None, size: int = 640):
        self.size = size
        if device is None:
            device = "CUDA" if torch.cuda.is_available() else "CPU"
        self.model = OnnxModel(
            path.join(ONNX, "{}.onnx".format(model)),
            providers=["{}ExecutionProvider".format(device.upper())]
        )

    def __call__(self, image: np.ndarray, threshold: float = 0.5) -> tuple:
        size = image.shape[:2]
        image, padding = self.preprocess(image)
        result = self.inference(image[np.newaxis])
        result = self.post_process(result, threshold, size, padding)
        return tuple(map(lambda tensor: tensor.numpy(), result))

    def resize(self, image: np.ndarray, pad: int = 114) -> tuple:
        h, w, _ = image.shape
        if max(h, w) < self.size:
            size = None
            width, height = (self.size - w) >> 1, (self.size - h) >> 1
        elif h > w:
            size = (round(self.size * w / h), self.size)
            width, height = (self.size - size[0]) >> 1, 0
        else:
            size = (self.size, round(self.size * h / w))
            height, width = (self.size - size[1]) >> 1, 0
        if size is not None:
            image = cv2.resize(image, size)
        return cv2.copyMakeBorder(
            image, height, height, width, width,
            cv2.BORDER_CONSTANT, value=(pad,) * 3
        ), (height, width)

    def inference(self, inputs: np.ndarray) -> torch.Tensor:
        return torch.tensor(self.model.run(None, {"images": inputs})[0])

    def preprocess(self, image: np.ndarray) -> tuple:
        image, pad = self.resize(image)
        image = image.transpose((2, 0, 1))
        return np.float32(image) / 255, pad

    def post_process(self,
                     outputs: torch.Tensor,
                     threshold: float,
                     size: tuple,
                     padding: tuple) -> tuple:
        outputs = outputs[outputs[..., 4] > threshold]
        classes = torch.argmax(outputs[:, 5:], 1)
        outputs[:, 5:] *= outputs[:, 4:5]
        keep = (scores := torch.max(outputs[:, 5:], 1).values) >= threshold
        outputs, classes, scores = outputs[keep], classes[keep], scores[keep]
        keep = classes == 0
        bboxes = outputs[keep, :4]
        bboxes[:, 0] -= padding[1]
        bboxes[:, 1] -= padding[0]
        bboxes[:, ::2] *= size[1] / (self.size - padding[1] * 2)
        bboxes[:, 1::2] *= size[0] / (self.size - padding[0] * 2)
        return bboxes.int(), classes[keep], scores[keep]

    @staticmethod
    def xyxy(x0y0wh: torch.Tensor) -> torch.Tensor:
        xyxy = torch.zeros_like(x0y0wh)
        xyxy[:, 0] = x0y0wh[:, 0] - x0y0wh[:, -2] // 2
        xyxy[:, 1] = x0y0wh[:, 1] - x0y0wh[:, -1] // 2
        xyxy[:, 2] = x0y0wh[:, 0] + x0y0wh[:, -2] // 2
        xyxy[:, 3] = x0y0wh[:, 1] + x0y0wh[:, -1] // 2
        return xyxy


class Detector:
    def __init__(self, node: str):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.color = None
        self.depth = None
        rospy.init_node(node)
        self.bridge = CvBridge()
        self.result = PoseStamped()
        self.detector = YOLOv6("YOLOv6")
        self.listener = tf.TransformListener()
        self.odom_frame = rospy.get_param("odom_frame", "odom")
        self.robot_frame = rospy.get_param("robot_frame", "base_link")
        self._1 = rospy.Subscriber(
            "/frame/color", Image,
            lambda image: self.msg2img(image, True)
        )
        self._2 = rospy.Subscriber(
            "/frame/depth", Image,
            lambda image: self.msg2img(image, False)
        )
        self.show = rospy.Publisher(
            "/frame/frame", Image, queue_size=1
        )
        self.publisher = rospy.Publisher(
            "/target", PoseStamped, queue_size=1
        )
        self.sleep = rospy.Rate(60).sleep
        self.intrinsics = np.linalg.inv(K)
        self.result.pose.orientation.w = 1
        self.result.header.frame_id = self.odom_frame
        self.listener.waitForTransform(
            self.odom_frame, self.robot_frame,
            rospy.Time(), rospy.Duration(10.0)
        )
        for topic in ("color", "depth"):
            rospy.wait_for_message("/frame/{}".format(topic), Image)
    
    def solve(self, x: int, y: int) -> None:
        z = self.depth[
            y-5:y+5, x-5:x+5
        ] * 1e-3
        z = np.mean(z[z != 0]) - 0.4
        sin, cos = np.sin(self.yaw), np.cos(self.yaw)
        x, y, z = z * np.dot(self.intrinsics, np.array([x, y, 1]).T).T
        self.result.pose.position.x = self.x + z * cos + x * sin  # x, y = z, -x
        self.result.pose.position.y = self.y + z * sin - x * cos  # x, y = z, -x
    
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

    def detection(self):
        while not rospy.is_shutdown():
            self.locate()
            frame = self.color.copy()
            bboxes, _, scores = self.detector(
                cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            )
            if bboxes.shape[0]:
                index = np.argmax(scores)
                if scores[index] >= 0.5:
                    (x, y, w, h), scores = bboxes[index], scores[index]
                    cv2.rectangle(
                        frame, (x - w // 2, y - h // 2),
                        (x + w // 2, y + h // 2), (0, 0xFF, 0), 2
                    )
                    self.solve(x, y)
                    self.publisher.publish(self.result)
            self.show.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            self.sleep()
        cv2.destroyAllWindows()


Detector("detector").detection()
