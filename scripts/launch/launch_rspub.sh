#!/bin/bash
# Launch RS-PUB
export OMP_NUM_THREADS=1
export FLAG_logbuflevel=-1
ARCH_USE="${ARCH:-x86_64}"  # If variable not set or null, use default.
./bin/${ARCH_USE}/rs-pub --log_dir=./logs --v=0 --config=config/l515/rspub_default.toml

