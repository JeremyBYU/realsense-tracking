#!/bin/bash
# Launch RS-PUB
cd /opt/workspace
export OMP_NUM_THREADS=1
./bin/${ARCH}/rs-pub --log_dir=./logs --v=0 --config=config/l515/rspub_default.toml
