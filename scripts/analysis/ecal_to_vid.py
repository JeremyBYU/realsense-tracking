import argparse
from pathlib import Path
from datetime import datetime
import sys
import time
import logging
import csv

import numpy as np
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber


THIS_FILE = Path(__file__)
THIS_DIR = THIS_FILE.parent

build_dir = THIS_DIR.parent.parent / f"dk-x86_64-build"
print(build_dir)
sys.path.insert(1, str(build_dir))
build_dir = THIS_DIR.parent.parent / "cmake-build"
sys.path.insert(1, str(build_dir))

from LandingMessage_pb2 import LandingMessage
from PoseMessage_pb2 import PoseMessage
from ImageMessage_pb2 import ImageMessage
from landing.helper.helper_utility import setup_frames, transform_pose
from moviepy.editor import ImageSequenceClip

logging.basicConfig(level=logging.INFO)





class EcalToVid(object):
    def __init__(self, fpath):
        self.setup_ecal()
        self.fath = fpath
        self.all_frames = []

    def callback_lm(self, topic_name, lm: LandingMessage, time_):
        print(lm, time_)

    def callback_rgbd_image(self, topic_name, image: ImageMessage, time_):
        logging.info("Received RGBD Image")
        im = np.frombuffer(image.image_data, dtype=np.uint8).reshape((image.height, image.width, 3))
        im = im[..., ::-1].copy()
        self.all_frames.append(im)

    def setup_ecal(self):

        ecal_core.initialize([], "ECALToCSV")
        # set process state
        ecal_core.set_process_state(1, 1, "Healthy")

        ecal_core.sub_destroy

        # create subscriber for depth information and connect callback
        self.sub_rgb_image = ProtoSubscriber("RGBDMessage", ImageMessage)
        self.sub_rgb_image.set_callback(self.callback_rgbd_image)


    def save_data(self):
        logging.info("Saving Info")
        my_clip = ImageSequenceClip(self.all_frames, fps=6)
        my_clip.write_videofile(self.fath,fps=6)



def parse_args():
    parser = argparse.ArgumentParser(
        description='Convert ECAL HDF5 RGBD to video'
    )

    parser.add_argument('--fpath', default='ecal_movie.mp4')

    args = parser.parse_args()
    print(args)
    return args


def main():
    args = parse_args()

    ecal_to_vid = EcalToVid(args.fpath)

    try:
        while ecal_core.ok():
            time.sleep(0.1)

    except KeyboardInterrupt:
        print()
        print("Ctrl-C detected, exiting loop")
        # finalize eCAL API
        ecal_core.finalize()
    # save data
    ecal_to_vid.save_data()


if __name__ == '__main__':
    main()
