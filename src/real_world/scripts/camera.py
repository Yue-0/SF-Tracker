import rospy
import pyrealsense2 as rs
from numpy import asarray
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

__author__ = "YueLin"


class RealSense:
    def __init__(self, fps: int = 30, width: int = 640, height: int = 480):
        self.camera, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.align = rs.align(rs.stream.color)
        self.camera.start(cfg)
    
    def read(self) -> map:
        frames = self.align.process(self.camera.wait_for_frames())
        return map(
            lambda frame: asarray(frame.get_data()),
            (frames.get_color_frame(), frames.get_depth_frame())
        )
    
    def close(self) -> None:
        self.camera.stop()


class Camera:
    def __init__(self, fps: int, node: str, w: int = 640, h: int = 480):
        rospy.init_node(node)
        self.sleep = rospy.Rate(fps).sleep
        self.camera = RealSense(fps, w, h)
        self.img2msg = CvBridge().cv2_to_imgmsg
        self.publishers = [
            rospy.Publisher("/frame/color", Image, queue_size=1),
            rospy.Publisher("/frame/depth", Image, queue_size=1)
        ]

    def run(self) -> None:
        while not rospy.is_shutdown():
            self.sleep()
            color, depth = self.camera.read()
            self.publishers[0].publish(self.img2msg(color, "bgr8"))
            self.publishers[1].publish(self.img2msg(depth, "mono16"))
        self.camera.close()


Camera(60, "camera").run()
