#!/bin/bash
# For some reason the rs-integrate service will ONLY work if launched as an entry point!
# https://github.com/continental/ecal/issues/238
cd /opt/workspace
export OMP_NUM_THREADS=1
echo "Delaying startup for 5 seconds"
sleep 5
./bin/${ARCH}/rs-integrate-server --log_dir=./logs --v=2 --config=config/l515/rsintegrate_default.toml
