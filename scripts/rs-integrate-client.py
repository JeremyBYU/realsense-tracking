import sys
from pathlib import Path
import time

import ecal.core.core as ecal_core
from ecal.core.service import Client

THIS_DIR = Path(__file__).parent
BUILD_DIR = (THIS_DIR / ".." / "build").resolve()
sys.path.insert(1, str(BUILD_DIR))


from Integrate_pb2 import SceneRequestType, DataRequestType, IntegrateRequest, IntegrateResponse, ADD

def callback(service_info, response):
    print(response)

def main():
      # initialize eCAL API
    ecal_core.initialize([], "Testing")
    
    # set process state
    ecal_core.set_process_state(1, 1, "Great")

    client = Client("IntegrateService")
    time.sleep(2)

    ir = IntegrateRequest()
    ir.scene = "Default"
    ir.type = ADD
    request_string = ir.SerializeToString()
    client.add_response_callback(callback)

    request_string = bytes("STUFF", 'ascii')
    print(request_string)
    res = client.call_method("IntegrateScene", request_string)
    print(res)
    # client.call_method("IntegrateScene",ir)


if __name__ == "__main__":
    main()
