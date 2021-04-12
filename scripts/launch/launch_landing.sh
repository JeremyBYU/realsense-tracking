#!/bin/bash
# For some reason the rs-integrate service will ONLY work if launched as an entry point!
# https://github.com/continental/ecal/issues/238
cd /opt/workspace
export OMP_NUM_THREADS=1
echo "Delaying startup for 1 seconds"
sleep 1
python -m landing
