import argparse
from pathlib import Path
from datetime import datetime
import sys
import time
import logging
import csv


import ecal
import ecal.core.core as ecal_core
import yaml
from ecal.core.subscriber import ProtoSubscriber
from scipy.spatial.transform import Rotation as R


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


logging.basicConfig(level=logging.INFO)


class EcalToCSV(object):
    def __init__(self, config,  save_dir):
        self.config = config
        self.save_dir = save_dir
        self.setup_ecal()
        self.frames = setup_frames(config)

        date = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")
        self.save_file = Path(save_dir) / f"poses_{date}.csv"
        self.pose_data = []
        print(self.save_file)

    def callback_lm(self, topic_name, lm: LandingMessage, time_):
        print(lm, time_)

    def callback_pose(self, topic_name, pose: PoseMessage, time_):
        logging.info("Got Pose Data: %s", len(self.pose_data))
        try:
            pose_raw_t = pose.translation
            pose_raw_r = pose.rotation
            H_body_w_ned = transform_pose(pose, self.frames['body_frame_transform_in_t265_frame'], self.frames['t265_world_to_ned_world'])
            ned_rot = R.from_matrix(H_body_w_ned[:3, :3]).as_euler('xyz', degrees=True)
            ned_pos = H_body_w_ned[:3, 3]
            self.pose_data.append(dict(pose_tx_raw=pose_raw_t.x, pose_ty_raw=pose_raw_t.y, pose_tz_raw=pose_raw_t.z,
                                    pose_rx_raw=pose_raw_r.x, pose_ry_raw=pose_raw_r.y, pose_rz_raw=pose_raw_r.z, pose_rw_raw=pose_raw_r.w,
                                    pose_tx_ned=ned_pos[0], pose_ty_ned=ned_pos[1], pose_tz_ned=ned_pos[2],
                                    pose_roll_ned=ned_rot[0], pose_pitch_ned=ned_rot[1], pose_yaw_ned=ned_rot[2]))

        except:
            logging.exception("Error")

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
        logging.info("Saving Info")
        with open(self.save_file, 'w', encoding='utf8', newline='') as output_file:
            fc = csv.DictWriter(output_file, fieldnames=self.pose_data[0].keys())
            fc.writeheader()
            fc.writerows(self.pose_data)


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
            time.sleep(0.1)

    except KeyboardInterrupt:
        print()
        print("Ctrl-C detected, exiting loop")
    # save data
    ecsv.save_data()
    # finalize eCAL API
    ecal_core.finalize()


if __name__ == '__main__':
    main()
