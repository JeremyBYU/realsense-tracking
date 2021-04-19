import argparse
from pathlib import Path
import sys
import ecal
import time
import ecal.core.core as ecal_core
import yaml
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


from landing.helper.helper_utility import setup_frames, transform_pose


class EcalToCSV(object):
    def __init__(self, config,  save_dir):
        self.config = config
        self.save_dir = save_dir
        self.setup_ecal()

        self.frames = setup_frames(config)
        print(self.frames)

        self.raw_pose_data = {}

    def callback_lm(self, topic_name, lm: LandingMessage, time_):
        print(lm, time_)

    def callback_pose(self, topic_name, pose: PoseMessage, time_):
        pose_ned = transform_pose(pose, self.frames['body_frame_transform_in_t265_frame'], self.frames['t265_world_to_ned_world'])

        # print(pose, time_)
        # print(pose_ned)

    def setup_ecal(self):

        ecal_core.initialize([], "ECALToCSV")
        # set process state
        ecal_core.set_process_state(1, 1, "Healthy")

        self.sub_pose = ProtoSubscriber("PoseMessage", PoseMessage)
        self.sub_pose.set_callback(self.callback_pose)

        # create subscriber for depth information and connect callback
        self.sub_lm = ProtoSubscriber("LandingMessage", LandingMessage)
        self.sub_lm.set_callback(self.callback_lm)


    def save_data(self):
        print("Will save data")


def parse_args():
    parser = argparse.ArgumentParser(
        description='Convert ECAL HDF5 to CSV for pose'
    )

    parser.add_argument('--config', '-c',
                        default="config/landing/landing.yml",
                        help='The config file',
                        )
    parser.add_argument('--data_dir', '-d',
                        default="assets/data",
                        help='The directory to save data',
                        )

    args = parser.parse_args()
    print(args)
    return args


def main():
    args = parse_args()

    with open(args.config) as file:
        config = yaml.safe_load(file)
    ecsv = EcalToCSV(config, args.data_dir)

    try:
        while ecal_core.ok():
            time.sleep(0.02)

    except KeyboardInterrupt:
        print()
        print("Ctrl-C detected, exiting loop")
    # save data
    ecsv.save_data()
    # finalize eCAL API
    ecal_core.finalize()


if __name__ == '__main__':
    main()
