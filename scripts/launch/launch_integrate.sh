#!/bin/bash
# For some reason the rs-integrate service will ONLY work if launched as an entry point!
# https://github.com/continental/ecal/issues/238
cd /opt/workspace
export OMP_NUM_THREADS=1
export FLAG_logbuflevel=-1
echo "Delaying startup for 7 seconds"
sleep 7
./bin/${ARCH}/rs-integrate-server --log_dir=./logs --v=2 --config=config/l515/rsintegrate_default.toml
