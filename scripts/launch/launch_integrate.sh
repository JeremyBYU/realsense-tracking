#!/bin/bash
# For some reason the rs-integrate service will ONLY work if launched as an entry point!
# https://github.com/continental/ecal/issues/238
export OMP_NUM_THREADS=1
export FLAG_logbuflevel=-1
ARCH_USE="${ARCH:-x86_64}"  # If variable not set or null, use default.
echo "Delaying startup for 5 seconds"
sleep 5
./bin/${ARCH_USE}/rs-integrate-server --log_dir=./logs --v=2 --config=config/l515/rsintegrate_default.toml
