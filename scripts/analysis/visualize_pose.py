import argparse
from pathlib import Path
from datetime import datetime
import sys
import time
import logging
import csv
import matplotlib.pyplot as plt


import pandas as pd
import numpy as np
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


t265_labels = ['hardware_ts', 'pose_tx_ned', 'pose_ty_ned', 'pose_tz_ned', 'pose_roll_ned', 'pose_pitch_ned', 'pose_yaw_ned']
gt_labels = ['hardware_ts', 'pose_tx_ned', 'pose_ty_ned', 'pose_tz_ned', 'pose_roll_ned', 'pose_pitch_ned', 'pose_yaw_ned']

def compare(config, compare_dir, gt_labels, fake=False, fname_t265='t265_pose.csv', fname_gt='gt_pose.csv'):
    # Read T265 CSV file
    df_t265 = pd.read_csv(Path(compare_dir) / fname_t265)
    df_t265_pose = df_t265[t265_labels]
    # Read GT or fake data
    if fake:
        df_gt = df_t265.copy()
        gt_labels = t265_labels
    else:
        df_gt = pd.read_csv(Path(compare_dir) / fname_gt)
    # Rename columns to be the same for gt and t265
    df_gt_pose = df_gt[gt_labels]
    rename_dict = dict()
    for (gt, other) in zip(gt_labels, t265_labels):
        rename_dict[gt] = other
    logging.info("Renaming columns %s", rename_dict)
    df_gt_pose.rename(columns=rename_dict)
    if fake:
        df_gt_pose = fake_data(df_gt_pose)

    logging.info("T265 Min timestamp: %d", df_t265_pose['hardware_ts'].min())
    logging.info("GT Min timestamp: %d", df_gt_pose['hardware_ts'].min())
    df_t265_pose.loc[:, ('hardware_ts')] = df_t265_pose['hardware_ts'] - df_t265_pose['hardware_ts'].min()
    df_gt_pose.loc[:, ('hardware_ts')] = df_gt_pose['hardware_ts'] - df_gt_pose['hardware_ts'].min()

    print(df_gt_pose)

    plot(df_t265_pose, df_gt_pose)

def plot(df_t265, df_gt, skip=10):
    df_a = df_t265.iloc[::skip, :]
    df_b = df_gt.iloc[::skip, :]
    fig, ax = plt.subplots(nrows=2, ncols=3, sharex=True, figsize=(15, 7) )

    t265_labels_matrix = np.array(t265_labels[1:]).reshape((2, 3))
    for i in range(t265_labels_matrix.shape[0]):
        for j in range(t265_labels_matrix.shape[1]):
            name = t265_labels_matrix[i,j]
            ax[i][j].plot(df_a['hardware_ts'] / 1000 / 1000, df_a[t265_labels_matrix[i,j]], label='T265')
            ax[i][j].plot(df_b['hardware_ts'] / 1000 / 1000, df_b[t265_labels_matrix[i,j]], label='GT')
            ax[i][j].set_title(name)
            ax[i][j].legend()

    plt.show()
def add_noise(df, column_name, mean=0.0, sigma=0.01):
    size = len(df[column_name])
    noise = mean + np.random.randn(size) * sigma
    df.loc[:, column_name] = df[column_name] + noise
    return df

def fake_data(df, offset_seconds=10):
    for name in t265_labels[1:4]:
        print(name)
        df = add_noise(df, name, sigma=0.001)

    for name in t265_labels[4:]:
        print(name)
        df = add_noise(df, name, mean=1.0, sigma=0.5)
    
    min_micro = df['hardware_ts'].min()
    offset_micro = int(offset_seconds * 1000 * 1000)
    start_micro = min_micro - offset_micro

    records = [dict(hardware_ts=i, pose_tx_ned=0.0, pose_ty_ned=0.0, pose_tz_ned=0.0, pose_roll_ned=0.0, pose_pitch_ned=0.0, pose_yaw_ned=0.0)  for i in range(start_micro, min_micro, 5000)]
    df_new = pd.DataFrame.from_records(records)
    df = pd.concat([df_new, df])
    return df

def parse_args():
    parser = argparse.ArgumentParser(
        description='Visualize Pose'
    )

    parser.add_argument('--config', '-c',
                        default="config/landing/landing.yml",
                        help='The config file',
                        )
    parser.add_argument('--compare_dir', '-cd',
                        default="assets/data/comp1",
                        help='The directory to compare',
                        )

    parser.add_argument('--fake', '-f', action='store_true', help='Fake gt data, adds noise to t265 data')
    parser.add_argument('-gt', '--ground_truth', nargs='+', type=str, default=gt_labels)

    args = parser.parse_args()
    print(args)
    return args


def main():
    args = parse_args()
    with open(args.config) as file:
        config = yaml.safe_load(file)

    compare(config, args.compare_dir, args.ground_truth, fake=args.fake)



if __name__ == '__main__':
    main()
