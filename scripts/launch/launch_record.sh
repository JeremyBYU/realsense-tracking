#!/bin/bash
# For some reason the rs-integrate service will ONLY work if launched as an entry point!
# https://github.com/continental/ecal/issues/238
timestamp=`date "+%Y%m%d-%H%M%S"` #add %3N as we want millisecond too
ecal_rec_client --whitelist "MeshAndTouchdownMessage,PoseMessage,RGBDLandingMessage,RGBDMessage,TouchdownMessage,LandingMessage" -d /home/jeremy/ecal_meas -n "$timestamp" -r