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

def callback_pose(topic_name, pose, time_):
    pass
    # print("")
    print(f"Pose translation: {pose.translation}; Pose HTS: {pose.hardware_ts:.2f}; Pose RTS: {time/1000:.2f}")


def callback_depth(topic_name, image, time_):
    global frame_count
    frame_count += 1

# def callback_rgbd(topic_name, image, time_):
#     global frame_count
#     frame_count += 1
#     t1 = time.perf_counter()
#     # print("")
#     print(f"Image Size: {image.width}X{image.height}; Pose HTS: {image.hardware_ts:.2f}; Pose RTS: {time_/1000:.2f}")

# define the server method "integrate_scene" function
def intregrate_scene_callback(method_name, req_type, resp_type, request):
    print("'DemoService' method '{}' called with {}".format(method_name, request))
    return 0, bytes("thank you for calling integrate :-)", "ascii")


def main():
    global frame_count
    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "Service_Server")
    # set process state
    ecal_core.set_process_state(1, 1, "Healthy")

    # create server "IntegrateService"
    server = ecal_service.Server("IntegrateService")
    server.add_method_callback("IntegrateScene",  "string", "string", intregrate_scene_callback)

    # create subscriber to pose information and connect callback
    sub_pose = ProtoSubscriber("PoseMessage", PoseMessage)
    sub_pose.set_callback(callback_pose)

    # create subscriber to depth information and connect callback
    sub_pose = ProtoSubscriber("PoseMessage", PoseMessage)
    sub_pose.set_callback(callback_pose)

    # create subscriber and connect callback
    # sub_rgbd = ProtoSubscriber("RGBDMessage", ImageMessage)
    # sub_rgbd.set_callback(callback_rgbd)


    frame_start = time.perf_counter()
    blocking_timer = time.perf_counter()
    while ecal_core.ok():
        rgbd_fps = frame_count / (time.perf_counter() - frame_start)
        print(f"RGDB FPS: {rgbd_fps:.1f}")
        if (time.perf_counter() - frame_start) > 10:
            frame_start = time.perf_counter()
            frame_count = 0
        # while(True):
        #     if (time.perf_counter() - blocking_timer) > 5.0:
        #         blocking_timer = time.perf_counter()
        #         break
        #     else:
        #         pass
        #         # print("blocking")
        time.sleep(5.0)

    # finalize eCAL API
    ecal_core.finalize()


if __name__ == "__main__":
    main()