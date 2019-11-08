#!/bin/bash
set -e
CMAKE_ARGS_ENV="" && [[ "$1" == "linux/arm64" ]]  && CMAKE_ARGS_ENV="-latomic"

