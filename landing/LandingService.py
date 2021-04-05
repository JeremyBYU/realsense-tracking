import sys
import time
from multiprocessing import Process, Lock, Queue, Manager
import multiprocessing
import queue
# from multiprocessing.managers import SyncManager, MakeProxyType, public_methods, BaseProxy
import numpy as np
import matplotlib.pyplot as plt
from landing.helper.helper_logging import logger
import open3d as o3d
from scipy.spatial.transform import Rotation as R

import ecal.core.core as ecal_core
import ecal.core.service as ecal_service
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher

from PoseMessage_pb2 import PoseMessage
from ImageMessage_pb2 import ImageMessage
from LandingMessage_pb2 import LandingMessage



from polylidar import Polylidar3D, MatrixDouble, MatrixFloat, extract_point_cloud_from_float_depth
from fastga import GaussianAccumulatorS2Beta, IcoCharts
from landing.helper.helper_polylidar import extract_polygons_from_points, get_3D_touchdown_point
from landing.helper.helper_utility import create_projection_matrix, create_transform, create_proto_vec
from landing.helper.o3d_util import create_o3d_pc, create_linemesh_from_shapely, get_segments
from landing.helper.helper_meshes import create_open_3d_mesh_from_tri_mesh
from landing.helper.helper_vis import plot_polygons


BLUE = (255, 0, 0)
class LandingService(object):
    def __init__(self, config):
        # This is our configuration variable, holds all parameters
        self.config = config
        # A simple frame counter
        self.frame_count = 0
        self.active_single_scan = True
        self.active_integration = False
        self.completed_integration = False
        self.pose_translation_ned = [0,0,0]
        self.pose_rotation_ned = [0,0,0,1]
        self.pose_touchdown_point = [0, 0, 0]
        self.pose_touchdown_dist = 0

        self.manager = Manager()  # manages shared data between polylidar process
        # These will contain the results of all single scan touchdowns points
        self.single_scan_touchdowns = self.manager.list()  # Lock is implicit when accessing between processes
        # These will contain the results of all integrated touchdown points
        self.integrated_touchdowns = self.manager.list()  # Lock is implicit when accessing between processes

        # Will set up all frames and fixed transformations
        self.setup_frames()
        # Wil set up ECAL communication
        self.setup_ecal()

    def callback_pose(self, topic_name, pose: PoseMessage, time_):
        # logger.info(f"Pose of T265 in T265 World Frame: translation: {vec3_to_str(pose.translation)}; \
        #                 rotation (deg): {np_to_str(t265_rot)}; Pose HTS: {pose.hardware_ts:.2f}; Pose RTS: {time_/1000:.2f}")
        try:
            # position and rotation of t265 in "t265 axis world frame" # see readme for definitions
            H_t265_w_t265 = create_transform([pose.translation.x, pose.translation.y, pose.translation.z],
                                             [pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w])
            t265_rot = R.from_matrix(H_t265_w_t265[:3, :3]).as_euler('xyz', degrees=True)
            logger.debug(
                f"Pose of T265 in T265 World Frame: translation: {vec3_to_str(pose.translation)}; rotation (deg): {np_to_str(t265_rot)}")

            H_body_w_t265 = H_t265_w_t265 @ self.body_frame_transform_in_t265_frame
            body_rot = R.from_matrix(H_body_w_t265[:3, :3]).as_euler('xyz', degrees=True)
            logger.debug(
                f"Pose of Body in T265 World Frame: translation: {np_to_str(H_body_w_t265[:3, 3])}; rotation (deg): {np_to_str(body_rot)};")
            # convert t265 axis world frame to ned world frame
            # Robot Modelling and Control, Spong, Similarity Transform 2.3.1, Page 41
            H_body_w_ned = self.t265_world_to_ned_world @ H_body_w_t265 @ np.linalg.inv(self.t265_world_to_ned_world)
            ned_rot = R.from_matrix(H_body_w_ned[:3, :3]).as_euler('xyz', degrees=True)
            logger.info(
                f"Pose of Body in NED World Frame: translation: {np_to_str(H_body_w_ned[:3, 3])}; rotation (deg): {np_to_str(ned_rot)}")
            
            self.pose_translation_ned = H_body_w_ned[:3, 3].flatten().tolist()

            # TODO - Write data to Matt??
        except Exception as e:
            logger.exception("Error!")
            # auto H_t265_W = make_transform(rotation, translation);
            # Eigen::Matrix4d extrinsic = (H_t265_d400.inverse() * H_t265_W * H_t265_d400).inverse();
        pass

    def callback_depth(self, topic_name, image: ImageMessage, time_):
        try:
            self.frame_count += 1
            if self.active_single_scan:
                self.landing_queue.put(image)
            # self.pub_image.send(image)
        except Exception as e:
            logger.exception("Error in callback depth")

    # define the server method "find_touchdown" function
    def callback_activate_single_scan_touchdown(self, method_name, req_type, resp_type, request):
        logger.info("'LandingService' method %s called with %s", method_name, request)
        # decodes bytes into utf-8 string
        request = request.decode('utf-8')
        self.active_single_scan = request == 'active'
        return 0, bytes("success", "ascii")

    def initiate_single_scan_landing(self):
        last_touchdown = self.single_scan_touchdowns[-1]
        tp = last_touchdown['touchdown_point']
        point = tp['point']
        dist = tp['dist']
        logger.info("Initiating landing at %s with %.2f radial clearance", point, dist)
        # TODO send command to beaglebone

    def initiate_integrated_landing(self):
        pass

    # define the server method "initiate_landing" function
    def callback_initiate_landing(self, method_name, req_type, resp_type, request):
        logger.info("'LandingService' method '%s' called with %s", method_name, request)
        request = request.decode('utf-8')
        rtn_value = 0
        rtn_msg = "success"
        if request == 'single_scan' and self.active_single_scan and len(self.single_scan_touchdowns) > 0:
            self.initiate_single_scan_landing()
            pass
        elif request == 'integrated' and self.integrated_complete:
            self.initiate_integrated_landing()
        else:
            rtn_value = 1
            rtn_msg = "invalid request. must have active single scan or completed integration"

        return rtn_value, bytes(rtn_msg, "ascii")

    def setup_frames(self):

        l515_mount = self.config['frames']['l515_sensor_mount']
        l515_axes = self.config['frames']['l515_sensor_axes']
        self.l515_to_drone_body = create_transform(np.array(l515_mount['translation']), l515_mount['rotation']) \
            @ create_transform(l515_axes['translation'], l515_axes['rotation'])

        t265_mount = self.config['frames']['t265_sensor_mount']
        self.t265_mount = create_transform(np.array(t265_mount['translation']), t265_mount['rotation'])
        t265_axes = self.config['frames']['t265_sensor_axes']
        self.t265_axes = create_transform(t265_axes['translation'], t265_axes['rotation'])
        self.t265_to_drone_body = self.t265_mount @ self.t265_axes

        # print(self.t265_mount)
        self.body_frame_transform_in_t265_frame = np.linalg.inv(self.t265_axes) @ self.t265_mount @ self.t265_axes
        self.body_frame_transform_in_t265_frame[:3, 3] = -self.body_frame_transform_in_t265_frame[:3, 3]
        # print()
        # print(self.body_frame_transform_in_t265_frame)
        # rotates world frame of t265 slam to world frame of drone axes
        self.t265_world_to_ned_world = self.t265_axes

        self.l515_to_t265_frame = np.linalg.inv(self.t265_to_drone_body) @ self.l515_to_drone_body

        # p_l515 = np.array([0, 0, 1, 1])
        # import ipdb;ipdb.set_trace()

    def setup_ecal(self):
        ecal_core.initialize(sys.argv, "Landing_Server")
        # set process state
        ecal_core.set_process_state(1, 1, "Healthy")

        # create server "LandingServer"
        self.server = ecal_service.Server("LandingService")
        # Will be lots of parameters inside of the request type
        self.server.add_method_callback("ActivateSingleScanTouchdown", "string", "string",
                                        self.callback_activate_single_scan_touchdown)
        self.server.add_method_callback("InitiateLanding", "string", "string", self.callback_initiate_landing)

        # create subscriber for pose information and connect callback
        self.sub_pose = ProtoSubscriber("PoseMessage", PoseMessage)
        self.sub_pose.set_callback(self.callback_pose)

        # create subscriber for depth information and connect callback
        self.sub_depth = ProtoSubscriber("RGBDMessage", ImageMessage)
        self.sub_depth.set_callback(self.callback_depth)

        # create publisher for depth information and connect callback
        self.pub_image = ProtoPublisher("RGBDLandingMessage", ImageMessage)
        self.pub_landing = ProtoPublisher("LandingMessage", LandingMessage)

        self.landing_queue = Queue()
        self.pub_image_queue = Queue()
        self.landing_process = Process(target=process_image, args=(
            self, self.landing_queue, self.pub_image_queue, self.single_scan_touchdowns))
        self.landing_process.daemon = True
        self.landing_process.start()        # Launch reader_proc() as a separate python proc

    def run(self):
        frame_start = time.perf_counter()

        while ecal_core.ok():
            rgbd_fps = self.frame_count / (time.perf_counter() - frame_start)
            # logger.info("RGBD FPS: %.1f", rgbd_fps)
            if (time.perf_counter() - frame_start) > 10:
                frame_start = time.perf_counter()
                self.frame_count = 0
            time.sleep(0.1)
            ## Publish Image Data
            try:
                image = self.pub_image_queue.get_nowait()
                self.pub_image.send(image)
            except queue.Empty:
                pass
            except:
                logger.exception("Error sending...")

            lm = LandingMessage()
            lm.active_single_scan = self.active_single_scan
            lm.active_integration = self.active_integration
            lm.pose_translation_ned.CopyFrom(create_proto_vec(self.pose_translation_ned))
            if len(self.single_scan_touchdowns) > 0:
                touchdown = self.single_scan_touchdowns[-1]
                if touchdown['touchdown_point'] is not None:
                    lm.single_touchdown_point.CopyFrom(create_proto_vec(touchdown['touchdown_point']['point'].tolist()))
            self.pub_landing.send(lm)
            

            # print(f"Main thread: {len(self.single_scan_touchdowns)}")

        # finalize eCAL API
        ecal_core.finalize()



def process_image(landing_service: LandingService, pull_queue: Queue, push_queue: Queue, single_scan_touchdowns):
    config = landing_service.config
    stride = config['mesh']['stride']

    # Create Polylidar Objects
    pl = Polylidar3D(**config['polylidar'])
    ga = GaussianAccumulatorS2Beta(level=config['fastga']['level'])
    ico = IcoCharts(level=config['fastga']['level'])

    while True:

        image = pull_queue.get()
        # logger.info("Frame Number: %s", image.frame_number)
        t1 = time.perf_counter()
        # Get numpy array from depth image
        image_depth_np = np.frombuffer(image.image_data_second, dtype=np.uint16).reshape((image.height, image.width))
        # Convert to float depth map
        image_depth_np = np.multiply(image_depth_np, landing_service.config['depth_scale'], dtype=np.float32)
        # Get intrinsics
        intrinsics = MatrixDouble(create_projection_matrix(image.fx, image.fy, image.cx, image.cy))
        # Get extrinsics for t265, in world t265 frame
        H_t265_w_t265 = create_transform([image.translation.x, image.translation.y, image.translation.z],
                                         [image.rotation.x, image.rotation.y, image.rotation.z, image.rotation.w])
        t2 = time.perf_counter()
        # Put in world ned frame
        extrinsics_world_ned = landing_service.t265_world_to_ned_world @ H_t265_w_t265 @ landing_service.l515_to_t265_frame
        extrinsics_world_ned_ = MatrixDouble(extrinsics_world_ned)
        extrinsics_body_frame_ = MatrixDouble(landing_service.l515_to_drone_body)
        # Put in frame chosen by user
        extrinsics = extrinsics_body_frame_ if config['single_scan']['command_frame'] == 'body' else extrinsics_world_ned_
        # Create OPC
        points = extract_point_cloud_from_float_depth(MatrixFloat(
            image_depth_np), intrinsics, extrinsics, stride=stride)
        new_shape = (int(image_depth_np.shape[0] / stride), int(image_depth_np.shape[1] / stride), 3)
        opc = np.asarray(points).reshape(new_shape)  # organized point cloud (will have NaNs!)
        t3 = time.perf_counter()
        chosen_plane, alg_timings, tri_mesh, avg_peaks, _ = extract_polygons_from_points(opc, pl, ga, ico, config)

        t4 = time.perf_counter()
        image_color_np = np.frombuffer(image.image_data, dtype=np.uint8).reshape((image.height, image.width, 3))
        if chosen_plane is not None:
            plot_polygons(chosen_plane, create_projection_matrix(image.fx, image.fy, image.cx,
                                                                 image.cy), np.linalg.inv(np.array(extrinsics)), image_color_np)
            touchdown_point = get_3D_touchdown_point(chosen_plane, config['polylabel']['precision'])
            plot_polygons((touchdown_point['circle_poly'], None), create_projection_matrix(image.fx, image.fy, image.cx, image.cy),
                          np.linalg.inv(np.array(extrinsics)), image_color_np, shell_color=BLUE)
            # VISUALIZATION
            # line_meshes = create_linemesh_from_shapely(chosen_plane[0])
            # tp = o3d.geometry.TriangleMesh.create_icosahedron(0.05).translate(touchdown_point['point'] )
            # o3d_mesh = create_open_3d_mesh_from_tri_mesh(tri_mesh)
            # o3d.visualization.draw_geometries([o3d_mesh, tp, *get_segments(line_meshes), o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)])
            # plt.imshow(image_color_np)
            # plt.show()

        else:
            touchdown_point = None
            logger.warn("Could not find a touchdown point")
        t5 = time.perf_counter()

        # Put modified image on queue to publish message
        image.image_data = np.ndarray.tobytes(image_color_np)
        push_queue.put(image)
        touchdown_results = dict(polygon=chosen_plane, alg_timings=alg_timings,
                                 avg_peaks=avg_peaks, touchdown_point=touchdown_point)
        single_scan_touchdowns.append(touchdown_results)

        # d1 = (t2 - t1) * 1000
        # d2 = (t3 - t2) * 1000
        # d3 = (t4 - t3) * 1000
        # d4 = (t5 - t4) * 1000
        # logger.info("D1: %.2f, D2: %.2f, D3: %.2f,  D4: %.2f, timings: %s", d1, d2, d3, d4, alg_timings)
        # print(f"Process thread: {len(single_scan_touchdowns)}")


def vec3_to_str(vec):
    return f"{vec.x:.1f}, {vec.y:.1f}, {vec.z:.1f}"


def np_to_str(vec):
    start = ""
    for x in vec:
        start += f"{x:.1f} "
    return start
