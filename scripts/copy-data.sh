#!/bin/bash
# exit when any command fails
set -e

# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
# echo an error message before exiting
trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT

SRC="."
DST_COMP="jeremy2@192.168.0.128"
DST_FOLDER="/home/jeremy2/ecal_meas"
file_name="$(basename -a $1)"
echo $file_name
echo "Attempting to copy data from $1 to $DST"
echo $1
scp -pr /home/jeremy/ecal_meas/$file_name $DST_COMP:$DST_FOLDER/$file_name
