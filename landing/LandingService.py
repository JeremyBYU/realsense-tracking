import sys
import time
from multiprocessing import Process, Lock, Queue
import numpy as np
import matplotlib.pyplot as plt
from landing.helper.helper_logging import logger

import ecal.core.core as ecal_core
import ecal.core.service as ecal_service
from ecal.core.subscriber import ProtoSubscriber

from PoseMessage_pb2 import PoseMessage
from ImageMessage_pb2 import ImageMessage


from polylidar import MatrixDouble, MatrixFloat, extract_point_cloud_from_float_depth

IDENTITY = np.identity(3)
IDENTITY_MAT = MatrixDouble(IDENTITY)


def process_image(landing_service, queue:Queue):
    while True:
        image = queue.get()
        image_np = np.frombuffer(image.image_data, dtype=np.uint16).reshape((image.height, image.width))

        depth_image = np.multiply(image_np, depth_scale, dtype=np.float32)
        points = extract_point_cloud_from_float_depth(MatrixFloat(
            depth_image), MatrixDouble(intrinsics), IDENTITY_MAT, stride=stride)
        image_np = image_np * landing_service.config['depth_scale']
        logger.info("Frame Number: %s", image.frame_number)

        # create opc



class LandingService(object):
    def __init__(self, config):
        self.config = config
        self.frame_count = 0
        self.setup()

    def callback_pose(self, topic_name, pose, time_):
        pass
        # print("")
        # print(f"Pose translation: {pose.translation}; Pose HTS: {pose.hardware_ts:.2f}; Pose RTS: {time_/1000:.2f}")

    def callback_depth(self, topic_name, image:ImageMessage, time_):
        try:
            self.landing_queue.put(image)
            self.frame_count += 1
        except Exception as e:
            logger.exception("Error in callback depth")

    # define the server method "find_touchdown" function
    def callback_find_touchdown(self, method_name, req_type, resp_type, request):
        logger.info("'LandingService' method %s called with %s",method_name, request)
        time.sleep(.3)
        return 0, bytes("thank you for calling touchdown :-)", "ascii")

    # define the server method "initiate_landing" function
    def callback_initiate_landing(self, method_name, req_type, resp_type, request):
        logger.info("'LandingService' method '%s' called with %s", method_name, request)
        return 0, bytes("thank you for calling initiate landing :-)", "ascii")

    def setup(self):
        ecal_core.initialize(sys.argv, "Landing_Server")
        # set process state
        ecal_core.set_process_state(1, 1, "Healthy")

        # create server "LandingServer"
        self.server = ecal_service.Server("LandingService")
        # Will be lots of parameters inside of the request type
        self.server.add_method_callback("FindTouchdown",  "string", "string", self.callback_find_touchdown)
        self.server.add_method_callback("InitiateLanding",  "string", "string", self.callback_initiate_landing)

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
            logger.info("RGBD FPS: %.1f", rgbd_fps)
            if (time.perf_counter() - frame_start) > 10:
                frame_start = time.perf_counter()
                self.frame_count = 0
            time.sleep(1.0)

        # finalize eCAL API
        ecal_core.finalize()
