import sys
import time
from pathlib import Path

import ecal.core.core as ecal_core
import ecal.core.service as ecal_service
from ecal.core.subscriber import ProtoSubscriber


# Its seems that it spawn a python THREAD for the server, and subsriber


THIS_FILE = Path(__file__)
THIS_DIR = THIS_FILE.parent
BUILD_DIR = THIS_DIR.parent.parent / "dk-x86_64-build"
print(BUILD_DIR.resolve())
sys.path.insert(1, str(BUILD_DIR))


from PoseMessage_pb2 import PoseMessage
from ImageMessage_pb2 import ImageMessage



frame_count = 0


# All subscription callbacks are in the same thread
# If one blocks, no messages will be recieved by the other SUBSCRIPTION CALLBACKS
#    does not affect subscriptions or main thread

# All Service callbacks are in the same thread
# If one blocks, no messages will be recieved by the other SERVICE CALLBACKS
#    does not affect subscriptions or main thread


def callback_pose1(topic_name, pose, time_):
    pass
    print("YO")
    # print("")
    print(f"Pose translation: {pose.translation}; Pose HTS: {pose.hardware_ts:.2f}; Pose RTS: {time_/1000:.2f}")

class LandingService(object):
    def __init__(self, config):
        self.frame_count = 0
        self.setup()
        self.run()

    def callback_pose(self, topic_name, pose, time_):
        pass
        # print("")
        # print(f"Pose translation: {pose.translation}; Pose HTS: {pose.hardware_ts:.2f}; Pose RTS: {time_/1000:.2f}")

    def callback_depth(self, topic_name, image, time_):
        self.frame_count += 1

    # define the server method "find_touchdown" function
    def callback_find_touchdown(self, method_name, req_type, resp_type, request):
        print("'LandingService' method '{}' called with {}".format(method_name, request))
        time.sleep(.3)
        return 0, bytes("thank you for calling touchdown :-)", "ascii")

    # define the server method "initiate_landing" function
    def callback_initiate_landing(self, method_name, req_type, resp_type, request):
        print("'LandingService' method '{}' called with {}".format(method_name, request))
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


    def run(self):
        frame_start = time.perf_counter()
        blocking_timer = time.perf_counter()
        while ecal_core.ok():
            rgbd_fps = self.frame_count / (time.perf_counter() - frame_start)
            print(f"RGDB FPS: {rgbd_fps:.1f}")
            if (time.perf_counter() - frame_start) > 10:
                frame_start = time.perf_counter()
                self.frame_count = 0
            time.sleep(1.0)

        # finalize eCAL API
        ecal_core.finalize()



def main():
    global frame_count
    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))

    LandingService({})
    # initialize eCAL API

    # need seperate process for communicating with serial connection



if __name__ == "__main__":
    main()