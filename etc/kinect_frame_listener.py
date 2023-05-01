# coding: utf-8

import numpy as np
import cv2
import sys
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel

try:
    from pylibfreenect2 import OpenGLPacketPipeline
    pipeline = OpenGLPacketPipeline()
except:
    try:
        from pylibfreenect2 import OpenCLPacketPipeline
        pipeline = OpenCLPacketPipeline()
    except:
        from pylibfreenect2 import CpuPacketPipeline
        pipeline = CpuPacketPipeline()
print("Packet pipeline:", type(pipeline).__name__)

# Create and set logger
logger = createConsoleLogger(LoggerLevel.Debug)
setGlobalLogger(logger)

fn = Freenect2()
num_devices = fn.enumerateDevices()
if num_devices == 0:
    print("No device connected!")
    sys.exit(1)

serial = fn.getDeviceSerialNumber(0)
device = fn.openDevice(serial, pipeline=pipeline)

listener = SyncMultiFrameListener(
    FrameType.Color | FrameType.Ir | FrameType.Depth)

# Register listeners
device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)

device.start()

# NOTE: must be called after device.start()
registration = Registration(device.getIrCameraParams(),
                            device.getColorCameraParams())

undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)

# Optinal parameters for registration
# set True if you need
need_bigdepth = False
need_color_depth_map = False

bigdepth = Frame(1920, 1082, 4) if need_bigdepth else None
# color_depth_map = np.zeros((424, 512),  np.int32).ravel() \
#     if need_color_depth_map else None
color_depth_map = np.zeros((424, 512), np.int32)
# if need_color_depth_map:
color_depth_map = color_depth_map.ravel()


# comment this out if you don't want to print the whole array
np.set_printoptions(threshold=np.inf)

i = 0  # counter to count how many output iterations will be run by the loop below

# while True:
while i < 1:
    frames = listener.waitForNewFrame()

    color = frames["color"]
    ir = frames["ir"]
    depth = frames["depth"]

    # print("color_depth_map shape:", color_depth_map.shape)
    # print("color_depth_map ndim:", color_depth_map.ndim)
    # color_depth_map = color_depth_map.reshape(-1)
    # print("color_depth_map shape:", color_depth_map.shape)
    # print("color_depth_map ndim:", color_depth_map.ndim)
    # color_depth_map = color_depth_map.astype(np.int32)
    # depth = depth.astype(np.int32)
    print("depth width: ", depth.width)
    print("depth height: ", depth.height)
    print("depth: ")
    print(depth.asarray())

    registration.apply(color, depth, undistorted, registered, enable_filter=True,
                       bigdepth=bigdepth, color_depth_map=color_depth_map)

    # print(color)

    # NOTE for visualization:
    # cv2.imshow without OpenGL backend seems to be quite slow to draw all
    # things below. Try commenting out some imshow if you don't have a fast
    # visualization backend.
    # cv2.imshow("ir", ir.asarray() / 65535.)
    cv2.imshow("depth", depth.asarray() / 4500.)
    # cv2.imshow("color", cv2.resize(color.asarray(),
    #                                (int(1920 / 3), int(1080 / 3))))
    cv2.imshow("registered", registered.asarray(np.uint8))

    print("registered shape: ", registered.asarray(np.uint8).shape)
    print("registered ndim: ", registered.asarray(np.uint8).ndim)
    print("registered: ")
    print(registered.asarray(np.uint8))

    # if need_bigdepth:
    #     cv2.imshow("bigdepth", cv2.resize(bigdepth.asarray(np.float32),
    #                                       (int(1920 / 3), int(1082 / 3))))
    # if need_color_depth_map:
    #     color_depth_map = cv2.normalize(
    #         color_depth_map, None, 0, 65535, cv2.NORM_MINMAX, cv2.CV_16U)
    # cv2.imshow("depth", color_depth_map)
    # cv2.imshow("color_depth_map", color_depth_map.reshape(424, 512))

    # print("color_depth_map", color_depth_map.reshape(424, 512))
    # print("color_depth_map shape:", color_depth_map.reshape(424, 512).shape)
    # print("color_depth_map ndim:", color_depth_map.reshape(424, 512).ndim)

    listener.release(frames)

    key = cv2.waitKey(delay=1)
    if key == ord('q'):
        break

    i += 1

# print(color)
device.stop()
device.close()

sys.exit(0)
