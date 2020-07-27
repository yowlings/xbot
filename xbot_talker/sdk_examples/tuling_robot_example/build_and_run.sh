#!/usr/bin/env bash

cd $(dirname $0)

rm -rf build || :
mkdir build && cd build

# 可以通过修改CC和CXX来设置编译器
CC=/usr/bin/gcc CXX=/usr/bin/g++ cmake ..
# cmake ..

make

echo "Running awaken demo ....."
./tuling_robot_demo
