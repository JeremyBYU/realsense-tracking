
import os
import sys
import time
from pathlib import Path
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
import logging

THIS_DIR = Path(__file__).parent
BUILD_DIR = (THIS_DIR / ".." / "build").resolve()

sys.path.insert(1, str(BUILD_DIR))
import ImageMessage_pb2, PoseMessage_pb2

logging.basicConfig(level=logging.INFO)

def callback(topic_name, image, send_ts):
    now = time.time() * 1000
    send_ts = send_ts / 1000
    logging.info("Received Depth Message; now: %.0f; send_ts: %.0f; hardware_ts: %.0f", now, send_ts, image.hardware_ts)


def main():
    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(),ecal_core.getdate()))

    # initialize eCAL API
    ecal_core.initialize(sys.argv, "RsSub")

    # set process state
    ecal_core.set_process_state(1, 1, "Healthy")

    # create subscriber and connect callback
    sub = ProtoSubscriber("DepthMessage", ImageMessage_pb2.ImageMessage)
    # sub = ProtoSubscriber("PoseMessage", PoseMessage_pb2.PoseMessage)
    sub.set_callback(callback)

    # idle main thread
    while ecal_core.ok():
        time.sleep(0.001)

    # finalize eCAL API
    ecal_core.finalize()
  
if __name__ == "__main__":
    main()