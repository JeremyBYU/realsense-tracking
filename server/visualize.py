
import os
import sys
import time
from pathlib import Path
import logging

import cv2
import numpy as np
import open3d as o3d
# from matplotlib.colors import Normalize
# import matplotlib.pyplot as plt
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber

THIS_DIR = Path(__file__).parent
BUILD_DIR = (THIS_DIR / ".." / "build").resolve()

sys.path.insert(1, str(BUILD_DIR))
import ImageMessage_pb2, PoseMessage_pb2, PointCloudMessage_pb2


from server.open3d_util import init_vis, handle_shapes, set_initial_view, get_extrinsics
from .integrator import Integrator

np.set_printoptions(suppress=True,
                    formatter={'float_kind': '{:.8f}'.format})


class Server(object):
    def __init__(self, config):
        super().__init__()
        self.set_up_callbacks()
        self.view_image = config['view_image']
        self.view_3D = config['view_3D']
        self.pointcloud = config['pointcloud']
        self.polylidar_kwargs = config['polygon']['polylidar']
        self.postprocess = config['polygon']['postprocess']
        self.record = config['record']
        self.n_points = config['n_points']

        vis, all_points, all_polys = init_vis(n_points=self.n_points, grid=dict(plane='xz', size=5, n=10))
        # vis.update_geometry(pcd)
        self.vis = vis
        self.all_points = all_points
        self.all_polys = all_polys
        self.integrator = Integrator(all_points=self.all_points)
        # stacked_str = "_stacked" if self.record['stacked'] and self.view_3D['active'] else ""
        # self.record['fpath'] = str(Path(self.record['directory']) / "{}_{}{}.avi".format(self.date, self.drive, stacked_str))

    def callback_depth(self, topic_name, image, send_ts):
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

    def callback_pose(self, topic_name, pose, send_ts):
        now = time.time() * 1000
        send_ts = send_ts / 1000
        logging.info("Received Pose Message; now: %.0f; send_ts: %.0f; hardware_ts: %.0f", now, send_ts, pose.hardware_ts)
        try:
            self.integrator.new_pose(pose)
            # logging.info("h %d; w: %d; bpp: %d; size_computed: %d, size: %d", h, w, bpp, h*w*bpp, depth.size)
        except Exception as e:
            print(e)
        # img = None

    def callback_points(self, topic_name, pc, send_ts):
        now = time.time() * 1000
        send_ts = send_ts / 1000
        n_points = pc.n_points
        logging.info("Received PC Message; now: %.0f; send_ts: %.0f; hardware_ts: %.0f; # Points: %d", now, send_ts, pc.hardware_ts, n_points)
        try:
            pc_data = pc.pc_data
            bpp = pc.bpp
            points = np.frombuffer(pc_data, dtype=np.float32).reshape((n_points,3))
            # Have RGB data
            if pc.format == 1:
                colors = np.frombuffer(pc.color_data, dtype=np.uint8).reshape((n_points, 3))
                colors = (colors / 255.0).astype('f8')
            else:
                colors = None
            # print(points[424 * 1, :])
            self.integrator.new_pc(points, colors, rotate=True, voxel_size=None)


        except Exception as e:
            print(e)

    def set_up_callbacks(self):
        logging.info("eCAL {} ({})\n".format(ecal_core.getversion(),ecal_core.getdate()))

        # initialize eCAL API
        ecal_core.initialize(sys.argv, "RSServer")

        # set process state
        ecal_core.set_process_state(1, 1, "Healthy")

        # create subscriber and connect callback
        # self.sub_depth = ProtoSubscriber("DepthMessage", ImageMessage_pb2.ImageMessage)
        # self.sub_depth.set_callback(self.callback_depth)

        self.sub_points = ProtoSubscriber("PointCloudMessage", PointCloudMessage_pb2.PointCloudMessage)
        self.sub_points.set_callback(self.callback_points)

        self.sub_poses = ProtoSubscriber("PoseMessage", PoseMessage_pb2.PoseMessage)
        self.sub_poses.set_callback(self.callback_pose)

    def run(self):

        # idle main thread
        while ecal_core.ok():
            self.vis.poll_events()
            self.vis.update_renderer()
            self.vis.update_geometry()
            time.sleep(0.1)

        # finalize eCAL API
        ecal_core.finalize()




def main():
    server = Server("")
    server.run()
  
if __name__ == "__main__":
    main()