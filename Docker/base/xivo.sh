#!/bin/bash
set -e
CMAKE_ARGS_ENV="" && [[ "$1" == "linux/arm64" ]]  && CMAKE_ARGS_ENV="-latomic"

git clone https://github.com/JeremyBYU/xivo.git /opt/xivo
# cd /opt/workspace/thirdparty/xivo
cd /opt/xivo
bash build.sh 

