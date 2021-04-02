import sys
import time
from multiprocessing import Process, Lock, Queue
import numpy as np
import matplotlib.pyplot as plt
from landing.helper.helper_logging import logger
import open3d as o3d
from scipy.spatial.transform import Rotation as R

import ecal.core.core as ecal_core
import ecal.core.service as ecal_service
from ecal.core.subscriber import ProtoSubscriber

from PoseMessage_pb2 import PoseMessage
from ImageMessage_pb2 import ImageMessage


from polylidar import Polylidar3D, MatrixDouble, MatrixFloat, extract_point_cloud_from_float_depth
from fastga import GaussianAccumulatorS2Beta, IcoCharts
from landing.helper.helper_polylidar import extract_polygons_from_points
from landing.helper.helper_utility import create_projection_matrix, create_transform
from landing.helper.o3d_util import create_o3d_pc

IDENTITY = np.identity(3)
IDENTITY_MAT = MatrixDouble(IDENTITY)


def process_image(landing_service, queue: Queue):
    config = landing_service.config
    stride = config['mesh']['stride']

    # Create Polylidar Objects
    pl = Polylidar3D(**config['polylidar'])
    ga = GaussianAccumulatorS2Beta(level=config['fastga']['level'])
    ico = IcoCharts(level=config['fastga']['level'])

    while True:

        image = queue.get()
        logger.info("Frame Number: %s", image.frame_number)

        # Get numpy array from image
        image_np = np.frombuffer(image.image_data, dtype=np.uint16).reshape((image.height, image.width))

        # Convert to float depth map
        image_np = np.multiply(image_np, landing_service.config['depth_scale'], dtype=np.float32)
        # Get intrinsics
        intrinsics = create_projection_matrix(image.fx, image.fy, image.cx, image.cy)
        # Get Extrinsics for body frame
        # Create OPC
        points = extract_point_cloud_from_float_depth(MatrixFloat(
            image_np), MatrixDouble(intrinsics), IDENTITY_MAT, stride=stride)

        new_shape = (int(image_np.shape[0] / stride), int(image_np.shape[1] / stride), 3)
        opc = np.asarray(points).reshape(new_shape)  # organized point cloud (will have NaNs!)

        # extract_polygons_from_points(opc, pl, ga, ico, config)
        # for visuazation
        # pcd =create_o3d_pc(opc)
        # o3d.visualization.draw_geometries([pcd])
        # create opc

def vec3_to_str(vec):
    return f"{vec.x:.1f}, {vec.y:.1f}, {vec.z:.1f}"

def np_to_str(vec):
    start = ""
    for x in vec:
        start += f"{x:.1f} "
    return start

class LandingService(object):
    def __init__(self, config):
        self.config = config
        self.frame_count = 0
        self.setup_frames()
        self.setup()

    def callback_pose(self, topic_name, pose:PoseMessage, time_):
        # logger.info(f"Pose of T265 in T265 World Frame: translation: {vec3_to_str(pose.translation)}; \
        #                 rotation (deg): {np_to_str(t265_rot)}; Pose HTS: {pose.hardware_ts:.2f}; Pose RTS: {time_/1000:.2f}")
        try:
            # position and rotation of t265 in "t265 axis world frame" # see readme for definitions
            H_t265_w_t265 = create_transform([pose.translation.x, pose.translation.y, pose.translation.z], 
                                        [pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w])
            t265_rot = R.from_matrix(H_t265_w_t265[:3,:3]).as_euler('xyz', degrees=True)
            logger.info(f"Pose of T265 in T265 World Frame: translation: {vec3_to_str(pose.translation)}; rotation (deg): {np_to_str(t265_rot)}")

            H_body_w_t265 = H_t265_w_t265 @ self.body_frame_transform_in_t265_frame
            body_rot = R.from_matrix(H_body_w_t265[:3,:3]).as_euler('xyz', degrees=True)
            logger.info(f"Pose of Body in T265 World Frame: translation: {np_to_str(H_body_w_t265[:3, 3])}; rotation (deg): {np_to_str(body_rot)};")
            # convert to t265 axis world frame to ned world frame
            # Robot Modelling and Control, Spong, Similarity Transform 2.3.1, Page 41 
            H_body_w_ned = self.t265_world_to_ned_world @ H_body_w_t265 @ np.linalg.inv(self.t265_world_to_ned_world) 
            ned_rot = R.from_matrix(H_body_w_ned[:3,:3]).as_euler('xyz', degrees=True)
            logger.info(f"Pose of Body in NED World Frame: translation: {np_to_str(H_body_w_ned[:3, 3])}; rotation (deg): {np_to_str(ned_rot)}")
            
            
        
        except Exception as e:
            logger.exception("Error!")
		# auto H_t265_W = make_transform(rotation, translation);
		# Eigen::Matrix4d extrinsic = (H_t265_d400.inverse() * H_t265_W * H_t265_d400).inverse();
        pass

    def callback_depth(self, topic_name, image: ImageMessage, time_):
        try:
            self.landing_queue.put(image)
            self.frame_count += 1
        except Exception as e:
            logger.exception("Error in callback depth")

    # define the server method "find_touchdown" function
    def callback_find_touchdown(self, method_name, req_type, resp_type, request):
        logger.info("'LandingService' method %s called with %s", method_name, request)
        time.sleep(.3)
        return 0, bytes("thank you for calling touchdown :-)", "ascii")

    # define the server method "initiate_landing" function
    def callback_initiate_landing(self, method_name, req_type, resp_type, request):
        logger.info("'LandingService' method '%s' called with %s", method_name, request)
        return 0, bytes("thank you for calling initiate landing :-)", "ascii")

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

        print(self.t265_mount)
        self.body_frame_transform_in_t265_frame = np.linalg.inv(self.t265_axes) @ self.t265_mount @ self.t265_axes
        self.body_frame_transform_in_t265_frame[:3, 3] = -self.body_frame_transform_in_t265_frame[:3, 3]
        print()
        print(self.body_frame_transform_in_t265_frame)
        # rotates world frame of t265 slam to world frame of drone axes 
        self.t265_world_to_ned_world = self.t265_axes

    def setup(self):
        ecal_core.initialize(sys.argv, "Landing_Server")
        # set process state
        ecal_core.set_process_state(1, 1, "Healthy")

        # create server "LandingServer"
        self.server = ecal_service.Server("LandingService")
        # Will be lots of parameters inside of the request type
        self.server.add_method_callback("FindTouchdown", "string", "string", self.callback_find_touchdown)
        self.server.add_method_callback("InitiateLanding", "string", "string", self.callback_initiate_landing)

        # create subscriber for pose information and connect callback
        self.sub_pose = ProtoSubscriber("PoseMessage", PoseMessage)
        self.sub_pose.set_callback(self.callback_pose)

        # create subscriber for depth information and connect callback
        self.sub_depth = ProtoSubscriber("DepthMessage", ImageMessage)
        self.sub_depth.set_callback(self.callback_depth)

        self.landing_queue = Queue()
        self.landing_process = Process(target=process_image, args=(self, self.landing_queue))
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
            time.sleep(1.0)

        # finalize eCAL API
        ecal_core.finalize()
