#!/bin/bash 
# Copy all shared library dependencies into the dependencies folder

./scripts/dependencies/cpld.sh ./bin/rs-pub ./dependencies
./scripts/dependencies/cpld.sh ./bin/rs-save ./dependencies
./scripts/dependencies/cpld.sh ./bin/rs-integrate-server ./dependencies

# This one seems to be missing
cp /usr/lib/libecaltime-localtime.so ./dependencies