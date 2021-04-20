import argparse
from pathlib import Path
from datetime import datetime
import sys
import time
import logging
import csv
import matplotlib.pyplot as plt
from itertools import product


import pandas as pd
import numpy as np
import ecal
import ecal.core.core as ecal_core
import yaml
from ecal.core.subscriber import ProtoSubscriber
from scipy.spatial.transform import Rotation as R
from scipy.signal import find_peaks
import open3d as o3d
import copy
from scipy.spatial.transform import Rotation as R



THIS_FILE = Path(__file__)
THIS_DIR = THIS_FILE.parent

build_dir = THIS_DIR.parent.parent / f"dk-x86_64-build"
sys.path.insert(1, str(build_dir))
build_dir = THIS_DIR.parent.parent / "cmake-build"
sys.path.insert(1, str(build_dir))

from LandingMessage_pb2 import LandingMessage
from PoseMessage_pb2 import PoseMessage
from landing.helper.helper_utility import setup_frames, transform_pose


logging.basicConfig(level=logging.INFO)


# csv labels
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

    # set to zero
    logging.info("T265 Min timestamp: %d", df_t265_pose['hardware_ts'].min())
    logging.info("GT Min timestamp: %d", df_gt_pose['hardware_ts'].min())
    df_t265_pose.loc[:, ('hardware_ts')] = df_t265_pose['hardware_ts'] - df_t265_pose['hardware_ts'].min()
    df_gt_pose.loc[:, ('hardware_ts')] = df_gt_pose['hardware_ts'] - df_gt_pose['hardware_ts'].min()

    print(df_gt_pose)
    # plot data before alignment
    logging.info("Before temporal alignment")
    plot(df_t265_pose, df_gt_pose)
    time_diff = find_time_matching_timestamps(df_t265_pose, df_gt_pose)
    df_gt_pose.loc[:, ('hardware_ts')] = df_gt_pose['hardware_ts'] - time_diff
    logging.info("After aligning the two data streams, time offset is: %d", time_diff)
    plot(df_t265_pose, df_gt_pose)

    return df_t265_pose, df_gt_pose

def find_time_matching_timestamps(df_t265, df_gt, field='pose_pitch_ned', skip=10,
                                    find_peaks_kwargs=dict(height=5, threshold=None, distance=10, prominence=9.0, wlen=100),
                                    gui=False):

    inv_scale = dict(pose_yaw_ned=1, pose_pitch_ned=-1)
    df_a = df_t265.iloc[::skip, :]
    df_b = df_gt.iloc[::skip, :]

    a = inv_scale[field] * df_a[field].to_numpy()
    b = inv_scale[field] * df_b[field].to_numpy()
    a_time = df_a['hardware_ts'].to_numpy()
    b_time = df_b['hardware_ts'].to_numpy()

    peaks_a, meta_a = find_peaks(a, **find_peaks_kwargs)
    peaks_b, meta_b = find_peaks(b, **find_peaks_kwargs)

    if gui:
        fig, ax = plt.subplots(nrows=1, ncols=2, sharex=False, figsize=(10, 5) )
        ax[0].plot(a)
        ax[0].plot(peaks_a, a[peaks_a], "x")

        ax[1].plot(b)
        ax[1].plot(peaks_b, b[peaks_b], "x")

        print(peaks_a, meta_a)
        print(peaks_b, meta_b)

        plt.show()
    
    time_diff = optimize_peak_linkage(a, peaks_a, meta_a, a_time, b, peaks_b, meta_b, b_time)
    return time_diff

def optimize_peak_linkage(a, peaks_a, meta_a, a_time,
                        b, peaks_b, meta_b, b_time, peak_thresh=5):
    combinations = list(product(peaks_a, peaks_b))

    a_peaks= dict()
    b_peaks = dict()
    possible_links = []
    # logging.info("All Combinations: %s", combinations)
    for idx_a, idx_b in combinations:
        val_a = a[idx_a]
        val_b = b[idx_b]
        diff = abs(val_b - val_a)
        if diff < peak_thresh:
            possible_links.append((idx_a, idx_b))
            if idx_a in a_peaks:
                a_peaks[idx_a].append(idx_b)
            else:
                a_peaks[idx_a] = [idx_b]
            if idx_b in b_peaks:
                b_peaks[idx_b].append(idx_a)
            else:
                b_peaks[idx_b] = [idx_a]

    time_diff = np.array([(b_time[idx_b] - a_time[idx_a])   for (idx_a, idx_b) in possible_links ])
    # print(possible_links, a_peaks, b_peaks)
    # print(time_diff)
    time_diff = np.median(time_diff)
    return time_diff



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

def fake_data(df, offset_seconds=10, gt_t_sigma=.001, gt_r_mean=1.0, gt_r_sigma=0.5):
    for name in t265_labels[1:4]:
        print(name)
        df = add_noise(df, name, sigma=gt_t_sigma)

    for name in t265_labels[4:]:
        print(name)
        df = add_noise(df, name, mean=gt_r_mean, sigma=gt_r_sigma)
    
    min_micro = df['hardware_ts'].min()
    offset_micro = int(offset_seconds * 1000 * 1000)
    start_micro = min_micro - offset_micro

    records = [dict(hardware_ts=i, pose_tx_ned=0.0, pose_ty_ned=0.0, pose_tz_ned=0.0, pose_roll_ned=0.0, pose_pitch_ned=0.0, pose_yaw_ned=0.0)  for i in range(start_micro, min_micro, 5000)]
    df_new = pd.DataFrame.from_records(records)
    df = pd.concat([df_new, df])
    df = df.reset_index()
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


def create_transform(pose):
    transform = np.eye(4)
    transform[:3, 3] = pose[:3]
    rot = R.from_euler('xyz', pose[3:], degrees=True).as_matrix()
    transform[:3,:3] = rot
    return transform


def box_center_to_corner(box):
    # To return
    corner_boxes = np.zeros((8, 3))

    translation = box[0:3]
    h, w, l = box[3], box[4], box[5]
    rotation = box[6]

    # Create a bounding box outline
    bounding_box = np.array([
        [-l/2, -l/2, l/2, l/2, -l/2, -l/2, l/2, l/2],
        [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2],
        [-h/2, -h/2, -h/2, -h/2, h/2, h/2, h/2, h/2]])

    # Standard 3x3 rotation matrix around the Z axis
    rotation_matrix = np.array([
        [np.cos(rotation), -np.sin(rotation), 0.0],
        [np.sin(rotation), np.cos(rotation), 0.0],
        [0.0, 0.0, 1.0]])

    # Repeat the [x, y, z] eight times
    eight_points = np.tile(translation, (8, 1))

    # Translate the rotated bounding box by the
    # original center position to obtain the final box
    corner_box = np.dot(
        rotation_matrix, bounding_box) + eight_points.transpose()

    return corner_box.transpose()

def create_bbox_to_ls(box):
    points = box_center_to_corner(box)

    lines = [[0, 1], [1, 2], [2, 3], [0, 3],
            [4, 5], [5, 6], [6, 7], [4, 7],
            [0, 4], [1, 5], [2, 6], [3, 7]]

    # Use the same color for all lines
    colors = [[1, 0, 0] for _ in range(len(lines))]

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set


def update_line_set(line_set, line_data, color=[1.0,0,0]):
    new_data = np.array(line_data)
    num_points = new_data.shape[0]

    lines = np.array([[i, i+1] for i in range(num_points -1)])
    colors = [color for _ in range(len(lines))]

    line_set.points = o3d.utility.Vector3dVector(new_data)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def plot_3d(df_t265, df_gt):

    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    box_np = np.array([0,0,0, 10, 10, 10, 0])
    box_ls = create_bbox_to_ls(box_np)

    line_t265 = []
    line_gt = []

    # create axis frames and line trace
    t265_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    t265_ls = o3d.geometry.LineSet()
    gt_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    gt_ls = o3d.geometry.LineSet()

    vert_copy = np.array(t265_frame.vertices).copy()

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Open3D', width=1920, height=1080,)
    vis.add_geometry(t265_frame)
    vis.add_geometry(gt_frame)
    vis.add_geometry(box_ls)
    vis.add_geometry(t265_ls)
    vis.add_geometry(gt_ls)


    time_gt = df_gt['hardware_ts'].to_numpy(dtype=np.int64)
    time_t265 = df_t265['hardware_ts'].to_numpy(np.int64)

    pose_gt = df_gt[t265_labels[1:]].to_numpy(dtype=np.float64)
    pose_t265 = df_t265[t265_labels[1:]].to_numpy(np.float64)

    min_gt = np.min(time_gt)
    min_t265 = np.min(time_t265)
    max_gt = np.max(time_gt)
    max_t265 = np.max(time_t265)

    min_value = np.min([min_gt, min_t265])
    max_value = np.max([max_gt, max_t265])

    transform_gt = np.eye(4)
    transform_t265 = np.eye(4)

    ms_step = 10
    us_step = ms_step * 1000
    sec_step = ms_step / 1000
    print(min_value, min_gt, min_t265)
    t1 = time.perf_counter()
    line_gt = []
    line_265 = []
    for t_value in range(0, max_value, us_step):
        if time.perf_counter() - t1 > sec_step:
            idx_gt = get_idx(t_value, time_gt)
            idx_t265 = get_idx(t_value, time_t265)
            t_gt = time_gt[idx_gt]
            t_t265 = time_t265[idx_t265]

            p_gt = pose_gt[idx_gt, :]
            p_t265 = pose_t265[idx_t265, :]

            line_gt.append(p_gt[:3])
            line_t265.append(p_t265[:3])

            gt_frame.vertices = o3d.utility.Vector3dVector(vert_copy.copy())
            t265_frame.vertices = o3d.utility.Vector3dVector(vert_copy.copy())

            update_line_set(t265_ls, line_t265, color=[1.0, 0.0, 0.0])
            update_line_set(gt_ls, line_gt, color=[0.0, 1.0, 0.0])

            transform_gt_new = create_transform(p_gt)
            transform_t265_new = create_transform(p_t265)

            if t_t265 < 0:
                transform_t265_new = np.eye(4)

            gt_frame.transform(transform_gt_new)
            t265_frame.transform(transform_t265_new)

            t265_frame.compute_triangle_normals()
            gt_frame.compute_triangle_normals

            # print(t_value)
            # print(idx_gt, idx_t265)
            # print(t_gt, t_t265)
            # print(p_gt, p_t265)
            # print()
            t1 = time.perf_counter()


        vis.update_geometry(gt_frame)
        vis.update_geometry(t265_frame)
        vis.update_geometry(t265_ls)
        vis.update_geometry(gt_ls)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(.005)


def get_idx(value, array, starting_idx=0, ending_idx=None):
    if ending_idx is None:
        ending_idx = array.shape[0]
    view = array[starting_idx:ending_idx]

    idx = np.searchsorted(view, value, side='left')
    return idx



def main():
    args = parse_args()
    with open(args.config) as file:
        config = yaml.safe_load(file)

    df_t265, df_gt = compare(config, args.compare_dir, args.ground_truth, fake=args.fake)
    plot_3d(df_t265, df_gt)



if __name__ == '__main__':
    main()
