
import os
import sys
import time
from pathlib import Path
import logging
import io

import cv2
import numpy as np

import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber

THIS_DIR = Path(__file__).parent
BUILD_DIR = (THIS_DIR / ".." / "build").resolve()

sys.path.insert(1, str(BUILD_DIR))
import ImageMessage_pb2, PoseMessage_pb2, PointCloudMessage_pb2

logging.basicConfig(level=logging.INFO)

def callback_depth(topic_name, image, send_ts):
    now = time.time() * 1000
    send_ts = send_ts / 1000
    logging.info("Received Depth Message; now: %.0f; send_ts: %.0f; hardware_ts: %.0f", now, send_ts, image.hardware_ts)
    try:
        image_data = image.image_data
        h = image.height
        w = image.width
        bpp = image.bpp
        depth = np.frombuffer(image_data, dtype=np.uint16).reshape((h,w))
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.1), cv2.COLORMAP_JET)
        cv2.imshow('Image', depth_colormap)
        cv2.waitKey(1)
        # logging.info("h %d; w: %d; bpp: %d; size_computed: %d, size: %d", h, w, bpp, h*w*bpp, depth.size)
    except Exception as e:
        print(e)
    # img = None

def callback_points(topic_name, pc, send_ts):
    now = time.time() * 1000
    send_ts = send_ts / 1000
    n_points = pc.n_points
    logging.info("Received PC Message; now: %.0f; send_ts: %.0f; hardware_ts: %.0f; # Points: %d", now, send_ts, pc.hardware_ts, n_points)
    try:
        pc_data = pc.pc_data
        bpp = pc.bpp
        points = np.frombuffer(pc_data, dtype=np.float32).reshape((n_points,3))

        # print(n_points, len(pc_data))
    except Exception as e:
        print(e)
    # img = None


def main():
    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(),ecal_core.getdate()))

    # initialize eCAL API
    ecal_core.initialize(sys.argv, "RsSub")

    # set process state
    ecal_core.set_process_state(1, 1, "Healthy")

    # create subscriber and connect callback
    sub_depth = ProtoSubscriber("DepthMessage", ImageMessage_pb2.ImageMessage)
    sub_depth.set_callback(callback_depth)

    sub_points = ProtoSubscriber("PointCloudMessage", PointCloudMessage_pb2.PointCloudMessage)
    sub_points.set_callback(callback_points)

    # idle main thread
    while ecal_core.ok():

        time.sleep(0.001)

    # finalize eCAL API
    ecal_core.finalize()
  
if __name__ == "__main__":
    main()