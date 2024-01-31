from time import sleep

import pyrealsense2 as rs

__author__ = "YueLin"

camera, cfg = rs.pipeline(), rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
cfg = camera.start(cfg)
sleep(1)

i = cfg.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
print(i)

camera.stop()
