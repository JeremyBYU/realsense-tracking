import time
import argparse
from pathlib import Path
import csv
import sys
import logging

from scipy.spatial.transform import Rotation as R
import numpy as np

MS_TO_NS = 1000000
sleep_time = 0.001

THIS_DIR = Path(__file__).parent
BUILD_DIR = THIS_DIR.parent / 'build'
sys.path.insert(0, str(BUILD_DIR.resolve()))

from PoseMessage_pb2 import PoseMessage, PoseMessageList
from PointCloudMessage_pb2 import PointCloudMessage
logging.basicConfig(level=logging.INFO)

class Converter(object):
    def __init__(self, data_dir):
        self.data_dir = data_dir
        self.pointcloud = self.data_dir / "pointcloud"
        self.scene = self.data_dir / "scene"
        self.color = self.data_dir / "color"

        self.pose_timestamps = None
        self.all_poses = None

        if self.pointcloud.exists():
            self.parse_point_clouds(self.pointcloud)

        if self.scene.exists():
            self.pose_timestamps, self.all_poses = self.parse_trajectory(
                self.scene)

        # if we have a color folder with frame_ids and timestamps in file name
        # use them to create a new trajectory log
        if self.color.exists():
            # these are the frame ids and associated timestamps
            frame_ids, timestamps = self.parse_frame_ids_and_ts(self.color)
            # will contain the poses at these specific timestamps
            frame_poses = []
            for ts_ in timestamps:
                idx = (np.abs(self.pose_timestamps - ts_)).argmin()
                frame_poses.append(self.all_poses[idx])
            logging.info("Writing trajectory.log for picture frames")
            self.write_poses(frame_poses, self.scene / "trajectory.log")

    def parse_point_clouds(self, folder: Path):
        logging.info("Processing Point Clouds")
        files = sorted(folder.glob("*.pb"))
        for file_ in files:
            print(file_)
            new_file = file_.with_suffix('.npz')
            with open(str(file_), 'rb') as f:
                pcm = PointCloudMessage()
                pcm.ParseFromString(f.read())
                pc_data = pcm.pc_data
                n_points = pcm.n_points
                bpp = pcm.bpp
                dim_ = 3 if pcm.format == 0 else 6
                points = np.frombuffer(
                    pc_data, dtype=np.float32).reshape((n_points, dim_))
                np.savez_compressed(str(new_file), points)

    @staticmethod
    def write_log(fh, counter, ht, ts=None):
        if ts is None:
            fh.write("{:d} {:d} {:d}\n".format(counter, counter, counter+1))
        else:
            fh.write("{:d} {:d} {:d} {:d}\n".format(
                counter, counter, counter+1, ts))
        fh.write("{:.8f} {:.8f} {:.8f} {:.8f}\n".format(*ht[0, :]))
        fh.write("{:.8f} {:.8f} {:.8f} {:.8f}\n".format(*ht[1, :]))
        fh.write("{:.8f} {:.8f} {:.8f} {:.8f}\n".format(*ht[2, :]))
        fh.write("{:.8f} {:.8f} {:.8f} {:.8f}\n".format(*ht[3, :]))

    def parse_frame_ids_and_ts(self, folder: Path):
        logging.info(
            "Processing Color Directory to get frame_ids and timestamps")
        files = sorted(folder.glob("*.jpg"))
        frame_ids = []
        timestamps = []

        for file_ in files:
            frame_id, ts = file_.stem.split('_')
            frame_ids.append(frame_id)
            timestamps.append(int(ts))

        frame_ids = np.array(frame_ids)
        timestamps = np.array(timestamps)
        return frame_ids, timestamps

    def write_poses(self, poses_list, fpath):
        with open(str(fpath), 'w') as f_new:
            for counter, pose in enumerate(poses_list):
                ts = int(pose.hardware_ts)
                ht = np.identity(4)
                r = R.from_quat([pose.rotation.x, pose.rotation.y,
                                 pose.rotation.z, pose.rotation.w])
                ht[:3, :3] = r.as_matrix()
                ht[:3, 3] = [pose.translation.x,
                             pose.translation.y, pose.translation.z]
                self.write_log(f_new, counter, ht, ts)

    def parse_trajectory(self, folder: Path):
        logging.info("Processing Trajectory")
        files = sorted(folder.glob("*.pb"))
        timestamps = []
        poses = []

        for file_ in files:
            print(file_)
            with open(str(file_), 'rb') as f:
                pml: PoseMessageList = PoseMessageList()
                pml.ParseFromString(f.read())
                for pose in pml.poses:
                    ts = int(pose.hardware_ts)
                    timestamps.append(ts)
                    poses.append(pose)

        raw_poses_fpath = folder / 'raw_trajectory.log'
        self.write_poses(poses, raw_poses_fpath)

        return np.array(timestamps), poses


def parse_args():
    parser = argparse.ArgumentParser(
        description='Convert RS Data'
    )

    parser.add_argument('--data_dir', '-d', action="store",
                        default="data/realsense",
                        help='The directory to read and save data',
                        )

    args = parser.parse_args()
    print(args)
    return args


def main():
    args = parse_args()
    data_dir = Path(args.data_dir)
    _ = Converter(data_dir)


if __name__ == "__main__":
    main()
