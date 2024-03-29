import argparse
from pathlib import Path
from datetime import datetime
import sys
import time
import logging
import csv
import ipdb
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
from itertools import product


import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
from scipy.signal import find_peaks
import open3d as o3d
import copy
from scipy.spatial.transform import Rotation as R

np.set_printoptions(precision=2, suppress=True)

THIS_FILE = Path(__file__)
THIS_DIR = THIS_FILE.parent

build_dir = THIS_DIR.parent.parent / f"dk-x86_64-build"
sys.path.insert(1, str(build_dir))
build_dir = THIS_DIR.parent.parent / "cmake-build"
sys.path.insert(1, str(build_dir))

# from landing.helper.helper_utility import setup_frames, transform_pose


logging.basicConfig(level=logging.INFO)

RAD_TO_DEG = 180 / np.pi


# csv labels
common_labels = ['hardware_ts', 'pose_tx_ned', 'pose_ty_ned', 'pose_tz_ned', 'pose_roll_ned', 'pose_pitch_ned', 'pose_yaw_ned']
rc_gt_labels = ['epoch_time_ns', 'X', 'Y', 'Z', 'roll', 'pitch', 'yaw']
rc_t265_labels = ['rsp_pose_time_us', 'rsp_pose_x', 'rsp_pose_y', 'rsp_pose_z', 'rsp_pose_roll', 'rsp_pose_pitch', 'rsp_pose_yaw']
# gt_labels = ['xbee_time_received_ns', 'xbee_x', 'xbee_y', 'xbee_z', 'xbee_roll', 'xbee_pitch', 'xbee_yaw']

def rename_data(label_a, label_b, df):
    rename_dict = dict()
    for (gt, other) in zip(label_a, label_b):
        rename_dict[gt] = other
    logging.info("Renaming columns %s", rename_dict)
    df = df.rename(columns=rename_dict)
    return df


def align_data(df_rc_gt, df_rc_t265):
    """Perforrms single rigid body transformation between ground truth and T265
    """
    gt_pose = df_rc_gt.iloc[0].to_numpy()
    est_pose = df_rc_t265.iloc[0].to_numpy()

    r_gt = R.from_euler('yxz', gt_pose[4:], degrees=True)
    r_est = R.from_euler('yxz', est_pose[4:], degrees=True)

    r_prime = r_gt.as_matrix() @ r_est.as_matrix().T
    t_prime = gt_pose[1:4] - r_prime @ est_pose[1:4]

    all_poses = df_rc_t265.iloc[:].to_numpy()[:, 1:4].T
    new_poses = r_prime @ all_poses + np.expand_dims(t_prime, axis=1)

    new_eulers = []
    for i in range(df_rc_t265.shape[0]):
        est_rot = df_rc_t265.iloc[i].to_numpy()[4:]
        r_est = R.from_euler('yxz', est_rot, degrees=True).as_matrix()
        new_rot_matrix = r_prime @ r_est
        new_euler = R.from_matrix(new_rot_matrix).as_euler('yxz', degrees=True)
        new_eulers.append(new_euler)
    
    new_eulers = np.array(new_eulers)
    
    df_rc_t265.iloc[:, 1:4] = new_poses.T
    df_rc_t265.iloc[:, 4:] = new_eulers



def compare(config, fpath='assets/data/analysis/flightlog/test06.csv', rc_gt_labels=rc_gt_labels, rc_t265_labels=rc_t265_labels,fake=False, time_match=False):
    """Compare between T265 and MCS. Align Data
    """
    # Read T265 CSV file
    df_rc = pd.read_csv(Path(fpath))
    df_rc = df_rc[df_rc.index % 2 != 0]  # Excludes every 2nd row starting from 0
    df_rc_gt = df_rc[rc_gt_labels]
    df_rc_t265 = df_rc[rc_t265_labels]

    # Rename columns to be the same for gt and t265
    df_rc_gt = rename_data(rc_gt_labels, common_labels, df_rc_gt)
    df_rc_t265 = rename_data(rc_t265_labels, common_labels, df_rc_t265)

    df_rc_gt = df_rc_gt.astype({'hardware_ts': 'uint64'})
    df_rc_t265 = df_rc_t265.astype({'hardware_ts': 'uint64'})

    df_rc_gt['pose_roll_ned'] = df_rc_gt['pose_roll_ned'] * RAD_TO_DEG
    df_rc_gt['pose_pitch_ned'] = df_rc_gt['pose_pitch_ned'] * RAD_TO_DEG
    df_rc_gt['pose_yaw_ned'] = df_rc_gt['pose_yaw_ned'] * RAD_TO_DEG
    df_rc_gt['hardware_ts'] = df_rc_gt['hardware_ts'].div(1000).astype(np.uint64)

    # Read GT or fake data for testing
    if fake:
        df_rc_t265_cp = df_rc_t265.copy()
        df_rc_gt = fake_data(df_rc_t265_cp, offset_seconds=0)
    
    
    # subtract to min
    df_gt_min = df_rc_gt['hardware_ts'].min()
    df_t265_min = df_rc_t265['hardware_ts'].min()
    df_rc_gt.loc[:, ('hardware_ts')] = df_rc_gt['hardware_ts'] - df_gt_min
    df_rc_t265.loc[:, ('hardware_ts')] = df_rc_t265['hardware_ts'] - df_t265_min

    logging.info("Plotting with complete message")
    plot(df_rc_t265, df_rc_gt, use_index=True)

    align_data(df_rc_gt, df_rc_t265)
    logging.info("Plotting with timestamps: After Alignment Adjustment")
    plot(df_rc_t265, df_rc_gt)
    if time_match:
        time_diff = find_time_matching_timestamps(df_rc_t265, df_rc_gt)
        logging.info("Plotting with timestamps: After Temporal Alignment; time offset is: %d", time_diff)
        df_rc_gt.loc[:, ('hardware_ts')] = df_rc_gt['hardware_ts'] - time_diff
        plot(df_rc_t265, df_rc_gt)
    else:
        df_rc_gt['hardware_ts'] = df_rc_t265['hardware_ts']

    return df_rc_t265, df_rc_gt

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


def update_axes(ax, meter=True):
    ax.set_xlabel('time (s)')
    ax.set_ylabel("meters" if meter else "degrees")

def plot(df_t265, df_gt, skip=10, use_index=False):
    "Plots data between T265 and GT (MCS)"

    plt.rcParams.update({'font.size': 14})
    df_a = df_t265.iloc[::skip, :]
    df_b = df_gt.iloc[::skip, :]
    fig, ax = plt.subplots(nrows=2, ncols=3, sharex=True, figsize=(15, 7) )

    map_label = dict(pose_tx_ned="X", pose_ty_ned="Y", pose_tz_ned="Z",pose_roll_ned='Roll', pose_pitch_ned='Pitch', pose_yaw_ned='Yaw')

    common_labels_matrix = np.array(common_labels[1:]).reshape((2, 3))
    for i in range(common_labels_matrix.shape[0]):
        for j in range(common_labels_matrix.shape[1]):
            name = common_labels_matrix[i,j]
            df_a_x = df_a['hardware_ts'] / 1000 / 1000
            df_b_x = df_b['hardware_ts'] / 1000 / 1000
            index_a = list(range(len(df_a[common_labels_matrix[i,j]])))
            index_b = list(range(len(df_b[common_labels_matrix[i,j]])))
            a_col = index_a if use_index else df_a_x
            b_col = index_b if use_index else df_b_x
            ax[i][j].plot(a_col, df_a[common_labels_matrix[i,j]], label='T265')
            ax[i][j].plot(b_col, df_b[common_labels_matrix[i,j]], label='MCS')
            ax[i][j].set_title(map_label[name])
            ax[i][j].legend()
            update_axes(ax[i][j], i == 0)
    plt.tight_layout()
    plt.show()

def add_noise(df, column_name, mean=0.0, sigma=0.01):
    size = len(df[column_name])
    noise = mean + np.random.randn(size) * sigma
    df.loc[:, column_name] = df[column_name] + noise
    return df

def fake_data(df, offset_seconds=1, gt_t_sigma=.001, gt_r_mean=1.0, gt_r_sigma=0.5):
    for name in common_labels[1:4]:
        print(name)
        df = add_noise(df, name, sigma=gt_t_sigma)

    for name in common_labels[4:]:
        print(name)
        df = add_noise(df, name, mean=gt_r_mean, sigma=gt_r_sigma)
    
    if offset_seconds > 0.1:
        min_micro = df['hardware_ts'].min()
        offset_micro = int(offset_seconds * 1000 * 1000)
        start_micro = int(min_micro - offset_micro) 

        # import ipdb; ipdb.set_trace()

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
    parser.add_argument('--fpath', '-fp',
                        default="assets/data/analysis/flightlog/test06.csv",
                        help='The directory to compare',
                        )

    parser.add_argument('--fake', '-f', action='store_true', help='Fake gt data, adds noise to t265 data')

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
    "Crates vertices for a box, box is [x,y,z,l,w.h,yaw]"
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
    "Converts a box to a lineset"
    points = box_center_to_corner(box)

    lines = [[0, 1], [1, 2], [2, 3], [0, 3],
            [4, 5], [5, 6], [6, 7], [4, 7],
            [0, 4], [1, 5], [2, 6], [3, 7]]

    # Use the same color for all lines
    colors = [[0.5, 0, 0.5] for _ in range(len(lines))]

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


def compute_ate(df_t265, df_gt):
    "Will comptue the absolute trajectory deviation"
    time_gt = df_gt['hardware_ts'].to_numpy(dtype=np.int64)
    time_t265 = df_t265['hardware_ts'].to_numpy(np.int64)

    pose_gt = df_gt[common_labels[1:]].to_numpy(dtype=np.float64)
    pose_t265 = df_t265[common_labels[1:]].to_numpy(np.float64)

    min_gt = np.min(time_gt)
    min_t265 = np.min(time_t265)
    max_gt = np.max(time_gt)
    max_t265 = np.max(time_t265)

    min_value = np.min([min_gt, min_t265])
    max_value = np.max([max_gt, max_t265])

    ms_step = 10
    us_step = ms_step * 1000
    sec_step = ms_step / 1000
    ate_rot = []
    ate_pos = []
    distance = []
    prior_pos = [0,0,0]
    for t_value in range(0, max_value, us_step):
        idx_gt = get_idx(t_value, time_gt)
        idx_t265 = get_idx(t_value, time_t265)

        gt_pose = pose_gt[idx_gt, :]
        est_pose = pose_t265[idx_t265, :]
        distance.append(np.linalg.norm(gt_pose[0:3] - prior_pos))
        prior_pos = gt_pose[0:3]
        r_gt = R.from_euler('yxz', gt_pose[3:], degrees=True).as_matrix()
        r_est = R.from_euler('yxz', est_pose[3:], degrees=True).as_matrix()

        delta_r_est = r_gt @ r_est.T
        delta_p = gt_pose[0:3] - delta_r_est @ est_pose[0:3]

        delta_r_degrees = np.degrees(np.linalg.norm(R.from_matrix(delta_r_est).as_rotvec()))
        ate_rot.append(delta_r_degrees)
        ate_pos.append(delta_p)

    ate_pos = np.array(ate_pos)
    ate_rot = np.array(ate_rot)

    ate_pos_norm = np.linalg.norm(ate_pos, axis=1)

    ate_rot_final = np.sqrt(np.mean(ate_rot ** 2))
    ate_pos_final = np.sqrt(np.mean(ate_pos_norm ** 2))

    total_distance = np.sum(distance)

    return ate_rot_final, ate_pos_final, total_distance



def plot_3d(df_t265, df_gt):
    "Plots the Drone trajectory using MCS and T265"

    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    box_np = np.array([0,0,0,0.05,3.5,3.5,0.0])
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

    pose_gt = df_gt[common_labels[1:]].to_numpy(dtype=np.float64)
    pose_t265 = df_t265[common_labels[1:]].to_numpy(np.float64)

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
    # print(min_value, min_gt, min_t265)
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

            update_line_set(t265_ls, line_t265, color=[0.12156863, 0.46666667, 0.70588235])
            update_line_set(gt_ls, line_gt, color=[1.0, 0.49803922, 0.05490196])

            transform_gt_new = create_transform(p_gt)
            transform_t265_new = create_transform(p_t265)

            if t_t265 < 0:
                transform_t265_new = np.eye(4)

            gt_frame.transform(transform_gt_new)
            t265_frame.transform(transform_t265_new)

            t265_frame.compute_triangle_normals()
            gt_frame.compute_triangle_normals
            t1 = time.perf_counter()


        vis.update_geometry(gt_frame)
        vis.update_geometry(t265_frame)
        vis.update_geometry(t265_ls)
        vis.update_geometry(gt_ls)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(.005)

    while(True):
        time.sleep(.005)
        vis.poll_events()
        vis.update_renderer()

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

    df_t265, df_gt = compare(config, args.fpath, fake=args.fake)
    ate_rot, ate_pos, total_distance = compute_ate(df_t265, df_gt)
    fname = Path(args.fpath).name
    print(f"File: {fname}; ATE_R: {ate_rot:.2f}; ATE_P: {ate_pos:.3f}; Length: {total_distance:.1f}m")
    plot_3d(df_t265, df_gt)



if __name__ == '__main__':
    main()
