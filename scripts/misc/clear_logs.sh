#!/bin/bash
find logs ! -name '.gitignore' \( -type l -o -type f \) -exec rm -f {} +