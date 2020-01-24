import sys
from pathlib import Path
import time

import ecal.core.core as ecal_core
from ecal.core.service import Client

# THIS_DIR = Path(__file__).parent
# BUILD_DIR = (THIS_DIR / ".." / "build").resolve()
# sys.path.insert(1, str(BUILD_DIR))


# from Integrate_pb2 import SceneRequestType, DataRequestType, IntegrateRequest, IntegrateResponse, ADD

def client_resp_callback(service_info, response):
    print(service_info)
    print(response)

def main():
      # initialize eCAL API
    ecal_core.initialize([], "Test_Integrate_Service")
    
    # set process state
    ecal_core.set_process_state(1, 1, "Healthy")

    client = Client("IntegrateService")

      # and add it to the client
    client.add_response_callback(client_resp_callback)
    time.sleep(2)

    while True:
        request_string = bytes("Call Integrate", "ascii")
        print(request_string)
        res = client.call_method("IntegrateScene", request_string)
        print(res)
        time.sleep(1.0)

if __name__ == "__main__":
    main()
