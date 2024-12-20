from time import sleep

import rospy
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

__author__ = "YueLin"


class RealSense:
    def __init__(self, fps: int = 30, width: int = 640, height: int = 480):
        # Setting up the camera
        self.camera, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        # Open the camera
        cfg = self.camera.start(cfg), sleep(1)

        # Depth map aligned to color map
        self.align = rs.align(rs.stream.color)

        # Get the camera's intrinsic matrix
        intrinsics = cfg[0].get_stream(
            rs.stream.color
        ).as_video_stream_profile().get_intrinsics()
        self.intrinsic = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy], [0, 0, 1]
        ])
    
    def read(self) -> map:
        """Read the current RGB image and depth image."""
        frames = self.align.process(self.camera.wait_for_frames())
        return map(
            lambda frame: np.asarray(frame.get_data()),
            (frames.get_color_frame(), frames.get_depth_frame())
        )
    
    def close(self) -> None:
        """Release camera."""
        self.camera.stop()


class Camera:
    def __init__(self, fps: int, node: str, w: int = 640, h: int = 480):
        """ROS camera node."""
        rospy.init_node(node)
        self.sleep = rospy.Rate(fps).sleep
        self.camera = RealSense(fps, w, h)
        self.img2msg = CvBridge().cv2_to_imgmsg
        self.publishers = [
            rospy.Publisher("/frame/color", Image, queue_size=1),
            rospy.Publisher("/frame/depth", Image, queue_size=1),
            rospy.Publisher("/intrinsic", Float32MultiArray, queue_size=1)
        ]
        self.intrinsic = Float32MultiArray()
        self.intrinsic.data = self.camera.intrinsic.flatten().tolist()

    def run(self) -> None:
        """Publish images and intrinsic"""
        while not rospy.is_shutdown():
            self.sleep()
            color, depth = self.camera.read()
            self.publishers[2].publish(self.intrinsic)
            self.publishers[0].publish(self.img2msg(color, "bgr8"))
            self.publishers[1].publish(self.img2msg(depth, "mono16"))
        self.camera.close()


# Main loop
Camera(60, "camera").run()
