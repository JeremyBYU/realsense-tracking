# pyrealsense2 is required.
# Please see instructions in https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python
from os import makedirs
import sys
import argparse
import sys
import time
import traceback
from os.path import exists, join
import shutil
import json
from enum import IntEnum

import pyrealsense2 as rs
import numpy as np
import cv2

sys.path.append("../Utility")
from file import write_poses_to_log

try:
    # Python 2 compatible
    input = raw_input
except NameError:
    pass

# T265 to T265
H_t265_d400 = np.array([
    [1, 0, 0, 0],
    [0, -1.0, 0, 0],
    [0, 0, -1.0, 0],
    [0, 0, 0, 1]])

POSES = []
POSES_TS = []


class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5


def quat_as_matrix(x, y, z, w, dim=4):
    x2 = x * x
    y2 = y * y
    z2 = z * z
    w2 = w * w

    xy = x * y
    zw = z * w
    xz = x * z
    yw = y * w
    yz = y * z
    xw = x * w

    matrix = np.identity(dim)

    matrix[0, 0] = x2 - y2 - z2 + w2
    matrix[1, 0] = 2 * (xy + zw)
    matrix[2, 0] = 2 * (xz - yw)

    matrix[0, 1] = 2 * (xy - zw)
    matrix[1, 1] = - x2 + y2 - z2 + w2
    matrix[2, 1] = 2 * (yz + xw)

    matrix[0, 2] = 2 * (xz + yw)
    matrix[1, 2] = 2 * (yz - xw)
    matrix[2, 2] = - x2 - y2 + z2 + w2

    return matrix

def make_clean_folder(path_folder):
    if not exists(path_folder):
        makedirs(path_folder)
    else:
        user_input = input("%s not empty. Overwrite? (y/n) : " % path_folder)
        if user_input.lower() == 'y':
            shutil.rmtree(path_folder)
            makedirs(path_folder)
        else:
            exit()


def save_intrinsic_as_json(filename, frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    with open(filename, 'w') as outfile:
        _ = json.dump(
            {
                'width':
                    intrinsics.width,
                'height':
                    intrinsics.height,
                'intrinsic_matrix': [
                    intrinsics.fx, 0, 0, 0, intrinsics.fy, 0, intrinsics.ppx,
                    intrinsics.ppy, 1
                ]
            },
            outfile,
            indent=4)


def process_depth(frames, align, filters, frame_count, path_depth, path_color, clipping_distance, args):
    # Align the depth frame to color frame
    # cframe = frames.get_color_frame()
    # if cframe is None:
    #     return None

    frames = align.process(frames)
    aligned_depth_frame = frames.get_depth_frame()
    for filter_ in filters:
        aligned_depth_frame = filter_.process(aligned_depth_frame)


    # Get aligned frames
    # aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    # Validate that both frames are valid
    if not aligned_depth_frame or not color_frame:
        return None

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    # color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

    if args.record_imgs:
        if frame_count == 0:
            save_intrinsic_as_json(
                join(args.output_folder, "camera_intrinsic.json"),
                color_frame)
        cv2.imwrite("%s/%06d.png" % \
                (path_depth, frame_count), depth_image)
        cv2.imwrite("%s/%06d.jpg" % \
                (path_color, frame_count), color_image)

    # Remove background - Set pixels further than clipping_distance to grey
    grey_color = 153
    #depth image is 1 channel, color is 3 channels
    depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
    bg_removed = np.where((depth_image_3d > clipping_distance) | \
            (depth_image_3d <= 0), grey_color, color_image)

    # Render images
    depth_colormap = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_image, alpha=0.09), cv2.COLORMAP_JET)
    images = np.hstack((bg_removed, depth_colormap))

    return images


def callback_pose(frame):
    global POSES, POSES_TS
    try:
        ts = frame.get_timestamp()
        domain = frame.frame_timestamp_domain
        pose = frame.as_pose_frame()
        data = pose.get_pose_data()
        t = data.translation
        r = data.rotation
        mat = quat_as_matrix(r.x, r.y, r.z, r.w, dim=4)
        mat[0, 3] = t.x; mat[1, 3] = t.y; mat[2, 3] = t.z
        mat = np.dot(mat,H_t265_d400)
        # Add poses to list
        POSES_TS.append(ts)
        POSES.append(mat)
        print("TS: {} - {:.1f}; Pose translation: {}".format(domain, ts, t))
    except Exception:
        print(traceback.print_exc())

def main():

    parser = argparse.ArgumentParser(
        description=
        "Realsense Recorder. Please select one of the optional arguments")
    parser.add_argument("--output_folder",
                        default='./dataset/realsense/',
                        help="set output folder")
    parser.add_argument("--record_rosbag",
                        action='store_true',
                        help="Recording rgbd stream into realsense.bag")
    parser.add_argument(
        "--record_imgs",
        action='store_true',
        help="Recording save color and depth images into realsense folder")
    parser.add_argument("--playback_rosbag",
                        action='store_true',
                        help="Play recorded realsense.bag file")
    args = parser.parse_args()

    if sum(o is not False for o in vars(args).values()) != 2:
        parser.print_help()
        exit()

    path_output = args.output_folder
    path_depth = join(args.output_folder, "depth")
    path_color = join(args.output_folder, "color")
    path_scene = join(args.output_folder, "scene")
    traj_fpath = join(path_scene, "trajectory.log")

    if args.record_imgs:
        make_clean_folder(path_output)
        make_clean_folder(path_depth)
        make_clean_folder(path_color)
        make_clean_folder(path_scene)

    path_bag = join(args.output_folder, "realsense.bag")
    if args.record_rosbag:
        if exists(path_bag):
            user_input = input("%s exists. Overwrite? (y/n) : " % path_bag)
            if user_input.lower() == 'n':
                exit()


    ctx = rs.context()
    dev_d400 = None
    dev_t265 = None
    pipe_d400 = None
    sensor_t265 = None
    cfg_d400 = None

    
    for dev in ctx.query_devices():
        dev_name = dev.get_info(rs.camera_info.name)
        print("Found {}".format(dev_name))
        if "Intel RealSense D4" in dev_name:
            dev_d400 = dev
        elif "Intel RealSense T265" in dev_name:
            dev_t265 = dev

    if dev_d400:
        cfg_d400 = rs.config()
        pipe_d400 = rs.pipeline()
        # Enable Streams
        cfg_d400.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        cfg_d400.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # Configure Depth sensor and get parameters
        depth_sensor = dev_d400.first_depth_sensor()
        depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_scale = depth_sensor.get_depth_scale()
        clipping_distance_in_meters = 3  # 3 meter
        clipping_distance = clipping_distance_in_meters / depth_scale
    if dev_t265:
        # Unable to open as a pipeline
        sensor_t265 = dev_t265.query_sensors()[0]
        profiles = sensor_t265.get_stream_profiles()
        pose_profile = [profile for profile in profiles if profile.stream_name() == 'Pose'][0]
        sensor_t265.open(pose_profile)
        sensor_t265.start(callback_pose)
        print("Starting T265 Pose Streaming")
    

    if dev_d400:
        pipe_d400.start(cfg_d400)
        print("Starting D4XX Color/Depth Streaming")


    # Using preset HighAccuracy for recording
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Streaming loop
    frame_count = 0
    images_ts = []
    frame_poses = []
    t0 = time.time()
    skip_time = 2
    filters = [rs.disparity_transform(True),rs.temporal_filter(0.2, 20.0, 2), rs.spatial_filter(0.5, 20.0, 2.0, 0.0), rs.disparity_transform(False)]
    try:
        cv2.namedWindow('Recorder Realsense', cv2.WINDOW_AUTOSIZE)
        while True:
            if dev_d400:
                d400_frames = pipe_d400.wait_for_frames()
                ts = d400_frames.get_timestamp()
                if (time.time() - t0) < skip_time:
                    continue
                # df = d400_frames.get_depth_frame()
                # print("Frame TS: {:.1f}".format(ts))
                # print(df.get_timestamp())
                # Global time stamps is not working, bug seems to exist only when using pipeline
                # print("Pose TS: {:.1f}".format(POSES_TS[-1]))
                # print(dir(d400_frames))
                domain = d400_frames.frame_timestamp_domain
                images_ts.append(ts)
                images = process_depth(d400_frames, align, filters, frame_count, path_depth, path_color, clipping_distance, args)
                print("TS: {} - {:.1f}; Saved color + depth image {:06d}".format(domain, ts, frame_count))
                if images is not None:
                    frame_count += 1
                    cv2.imshow('Recorder Realsense', images)
                    if POSES:
                        frame_poses.append(POSES[-1])
                key = cv2.waitKey(1)

                # if 'esc' button pressed, escape loop and exit program
                if key == 27 or key == 'q':
                    cv2.destroyAllWindows()
                    break
    finally:
        if dev_d400:
            pipe_d400.stop()
            time.sleep(0.1)
        if dev_t265:
            sensor_t265.stop()
            sensor_t265.close()
        print("Closed all sensors!")
    
    write_poses_to_log(traj_fpath, frame_poses)
    



if __name__ == "__main__":
    main()
