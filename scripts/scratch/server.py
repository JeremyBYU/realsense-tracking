import sys
import time

import ecal.core.core as ecal_core
import ecal.core.service as ecal_service

def main():
  # print eCAL version and date
  print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
  
  # initialize eCAL API
  ecal_core.initialize(sys.argv, "Service_Server")
  
  # set process state
  ecal_core.set_process_state(1, 1, "Healthy")

  # create server "IntegrateService"
  server = ecal_service.Server("IntegrateService")

  # define the server method "integrate_scene" function
  def intregrate_scene_callback(method_name, req_type, resp_type, request):
    print("'DemoService' method '{}' called with {}".format(method_name, request))
    return 0, bytes("thank you for calling integrate :-)", "ascii")


  # define the server methods and connect them to the callbacks
  server.add_method_callback("IntegrateScene",  "string", "string", intregrate_scene_callback)

  while ecal_core.ok():
      time.sleep(0.01)

  server.destroy()
  
  # finalize eCAL API
  ecal_core.finalize()

if __name__ == "__main__":
  main()