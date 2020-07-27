#!/bin/bash

# To run do:
# wget http://.../bootstrap.sh && chmod +x bootstrap.sh && ./bootstrap.sh && source ~/.profile
#
# This script will:
# * install sox
# * isntall libsox-fmt-all
# * install libgflags-dev

echo "Start installing dependencies:"
sudo apt install sox
sudo apt install libsox-fmt-all
sudo apt-get install libgflags-dev

echo "All dependencies have been installed sucessfully!"
