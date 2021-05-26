#!/bin/bash 
# Copy all shared library dependencies into the dependencies folder

# exit when any command fails
set -e
# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
# echo an error message before exiting
trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT

architectures="x86_64"
bin_dir="bin/"
dist_dir="bin/"

# Iterate the string variable using for loop
for val in $architectures; do
    echo $val
    bin_path="$bin_dir/$val"
    out_path="$dist_dir/$val"
    # mkdir -p out_path
    ./scripts/dependencies/cpld.sh $bin_path/rs-pub $out_path
    ./scripts/dependencies/cpld.sh $bin_path/rs-save $out_path
    ./scripts/dependencies/cpld.sh $bin_path/rs-integrate-server $out_path
    cp /usr/local/lib/libecaltime-localtime.so $out_path
done


# Example how to launch for x86_64
# GLOG_logtostderr=1 ./bin/x86_64/ld-linux-x86-64.so.2 ./bin/x86_64/rs-integrate-server --log_dir=./logs --v=2 --config=config/l515/rsintegrate_default.toml
