#!/bin/bash
# Launch RS-PUB and RS-Integrate
export OMP_NUM_THREADS=1
export LD_LIBRARY_PATH=./bin/x86_64
parallel --verbose --linebuffer ::: scripts/launch/launch_rspub.sh scripts/launch/launch_integrate.sh

