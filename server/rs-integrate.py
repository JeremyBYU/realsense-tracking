
import os
import sys
import time
from pathlib import Path
import logging
from typing import Dict

import cv2
import numpy as np
import open3d as o3d
import yaml
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.service import Server
from scipy.spatial.transform import Rotation as R

# THIS_DIR = Path(__file__).parent
# BUILD_DIR = (THIS_DIR / ".." / "build").resolve()
# sys.path.insert(1, str(BUILD_DIR))

from .rspub_pb import PointCloudMessage_pb2
from .rspub_pb import PoseMessage_pb2
from .rspub_pb import ImageMessage_pb2
from .rspub_pb.Integrate_pb2 import SceneRequestType, DataRequestType, IntegrateRequest, IntegrateResponse

from server.open3d_util import init_vis, handle_shapes, set_initial_view, get_extrinsics

np.set_printoptions(suppress=True,
                    formatter={'float_kind': '{:.8f}'.format})

DEFAULT_SCENE_KWARGS = dict(voxel_length=16.0 / 512.0,
                            sdf_trunc=0.08,
                            depth_sampling_stride=4,
                            color_type=o3d.integration.TSDFVolumeColorType.RGB8)


Scene = Dict[str, o3d.integration.ScalableTSDFVolume]


H_t265_d400 = np.array([
    [1, 0, 0, 0],
    [0, -1.0, 0, 0],
    [0, 0, -1.0, 0],
    [0, 0, 0, 1]])

R_World_d400 = np.identity(4)
R_World_d400[:3, :3] = R.from_euler('x', 90, degrees=True).as_matrix()

logging.basicConfig(level=logging.INFO)


def diff_pose(last_pose, last_quat, current_pose, current_quat):
    pose_changed = np.linalg.norm(last_pose - current_pose)
    quat_changed = current_quat @ last_quat
    quat_changed = quat_changed * quat_changed
    quat_changed = 1.0 - quat_changed

    return pose_changed, quat_changed


# define the server method "foo" function
def foo_req_callback(method_name, req_type, resp_type, request):
    print("'DemoService' method '{}' called with {}".format(method_name, request))
    return 0, bytes("thank you for calling foo :-)", "ascii")

class IntegrateServer(object):
    def __init__(self, config):
        super().__init__()
        # self.set_up_callbacks()
        self.set_up_service()
        return
        self.polylidar_kwargs = config['polygon']['polylidar']
        self.postprocess = config['polygon']['postprocess']
        self.depth_trunc = config.get('depth_trunc', 3.0)

        self.min_translate_change = config.get("min_translate_change", 0.1)
        self.min_rotation_change = config.get("min_rotation_change", 20.0)
        self.min_rotation_change = (
            1.0 - np.cos(np.deg2rad(self.min_rotation_change))) / 2.0

        vis, all_points, all_polys = init_vis(
            n_points=1, grid=dict(plane='xz', size=5, n=10))

        self.scalable_tsdf_kwargs = config.get(
            'scalable_tsdf', DEFAULT_SCENE_KWARGS)
        self.scalable_tsdf_kwargs['color_type'] = o3d.integration.TSDFVolumeColorType(
            self.scalable_tsdf_kwargs['color_type'])

        self.vis = vis
        self.all_points = all_points
        self.all_polys = all_polys
        self.last_position = np.zeros((3,))
        self.last_quat = np.ones((4,))

        self.new_mesh = None
        self.old_mesh = None

        # Read camera intrinsic
        if config.get("path_intrinsic"):
            self.intrinsic = o3d.io.read_pinhole_camera_intrinsic(
                config["path_intrinsic"])
        else:
            self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)

        self.scenes: Scene = dict()

    def add_scene(self, scene_name: str, scene_kwargs=None):
        """Creates a new scene volume which will be integrated

        Arguments:
            scene_name {str} -- [description]
            volume {[type]} -- [description]
        """
        scene_kwargs = scene_kwargs if scene_kwargs is not None else self.scalable_tsdf_kwargs
        volume = o3d.integration.ScalableTSDFVolume(**scene_kwargs)
        if self.scenes.get(scene_name):
            raise ValueError("Scene already exists! Remove first")
        self.scenes[scene_name] = volume

    def integrate_rgbd(self, scene_name: str, rgbd_image: o3d.geometry.RGBDImage, extrinsic: np.ndarray):
        """Integrate RGBD Image into a scene

        Arguments:
            scene_name {str} -- Scene Name
            rgbd_image {o3d.geometry.RGBDImage} -- RGBD Image
            extrinsic {np.ndarray} -- 4x4 numpy array

        Raises:
            ValueError: Invalid scene name
        """
        volume = self.scenes.get(scene_name)
        if volume is None:
            raise ValueError("Scene does not exist!")
        t0 = time.perf_counter()
        volume.integrate(rgbd_image, self.intrinsic, extrinsic)
        t1 = time.perf_counter()
        logging.info("RGBD Integration took: %.1f ms", (t1 - t0) * 1000)

    def extract_mesh(self, scene_name: str = "Default"):
        volume = self.scenes.get(scene_name)
        if volume is None:
            raise ValueError("Scene does not exist!")
        t0 = time.perf_counter()
        mesh = volume.extract_triangle_mesh()
        t1 = time.perf_counter()
        logging.info("Mesh Extraction took: %.1f ms", (t1 - t0) * 1000)
        return mesh

    def callback_rgbd(self, topic_name, image, send_ts):
        now = time.time() * 1000
        send_ts = send_ts / 1000
        logging.debug("Received RGBD Message; now: %.0f; send_ts: %.0f; hardware_ts: %.0f",
                      now, send_ts, image.hardware_ts)
        try:
            color_data = image.image_data
            depth_data = image.image_data_second
            h = image.height
            w = image.width

            # Get transform data if available
            quat = image.rotation
            quat = np.array([quat.x, quat.y, quat.z, quat.w])
            trans = image.translation
            rot = R.from_quat(quat).as_matrix()
            trans = np.array([trans.x, trans.y, trans.z])
            H_t265_W = np.identity(4)
            H_t265_W[:3, :3] = rot
            H_t265_W[:3, 3] = trans

            # Check if position changed sufficiently to integrate data
            translate_changed, rot_changed = diff_pose(
                self.last_position, self.last_quat, trans, quat)
            pose_changed = translate_changed > self.min_translate_change or rot_changed > self.min_rotation_change
            if not pose_changed:
                return
            # Pose is sufficiently different, update position and quat
            logging.info("Pose changed sufficiently")
            self.last_position = trans
            self.last_quat = quat

            # Create extrinsic
            extrinsic = np.linalg.inv(H_t265_d400) @ H_t265_W @ H_t265_d400
            extrinsic = np.linalg.inv(extrinsic)

            # Create Open 3D Images
            color = np.frombuffer(
                color_data, dtype=np.uint8).reshape((h, w, 3))
            depth = np.frombuffer(depth_data, dtype=np.uint16).reshape((h, w))

            color = np.ascontiguousarray(color[...,::-1])

            color = o3d.geometry.Image(color)
            depth = o3d.geometry.Image(depth)
            # print(color)
            # print(depth)
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color, depth, depth_trunc=self.depth_trunc, convert_rgb_to_intensity=False)

            self.integrate_rgbd("Default", rgbd, extrinsic)
            self.new_mesh = self.extract_mesh("Default")

            # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.1), cv2.COLORMAP_JET)
            # cv2.imshow('Image', color)
            # cv2.waitKey(1)
        except Exception as e:
            logging.exception("Error")

    def set_up_callbacks(self):
        logging.info("eCAL {} ({})\n".format(
            ecal_core.getversion(), ecal_core.getdate()))

        # initialize eCAL API
        ecal_core.initialize(sys.argv, "RSIntegrate")

        # set process state
        ecal_core.set_process_state(1, 1, "Healthy")

        self.sub_rgbd = ProtoSubscriber(
            "RGBDMessage", ImageMessage_pb2.ImageMessage)
        self.sub_rgbd.set_callback(self.callback_rgbd)


    @staticmethod
    def rpc_integrate_scene(method_name, req_type, resp_type, request):
        print(method_name)
        print(req_type)
        print(resp_type)
        print(request)
        return 0, bytes("Whats up", "ascii")

    def set_up_service(self):
                # initialize eCAL API
        ecal_core.initialize([], "RSIntegrate")

        # set process state
        ecal_core.set_process_state(1, 1, "Healthy")
        rpc_server = Server("IntegrateService")
        rpc_server.add_method_callback("IntegrateScene", "string", "string", foo_req_callback)

        self.rpc_server = rpc_server

    def run_server(self):

        while ecal_core.ok():
            # print("Here")
            time.sleep(0.1)

    def run(self):
        self.add_scene("Default")

        # idle main thread
        while ecal_core.ok():
            self.vis.poll_events()
            # self.vis.update_geometry(*self.all_points, reset_bounding_box=False)
            self.vis.update_renderer()
            if self.new_mesh is not None:
                if self.old_mesh is not None:
                    self.vis.remove_geometry(
                        self.old_mesh, reset_bounding_box=False)
                self.vis.add_geometry(self.new_mesh, reset_bounding_box=False)
                self.old_mesh = self.new_mesh
                self.new_mesh = None

            time.sleep(0.1)

        ecal_core.finalize()


def main():
    try:
        with open("./config/rsserver_default.yaml", 'r') as f:
            config = yaml.safe_load(f)
    except yaml.YAMLError:
        logging.exception("Error parsing yaml")

    server = IntegrateServer(config)
    server.run_server()


if __name__ == "__main__":
    main()
