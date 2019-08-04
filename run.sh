#! /bin/bash
cd ./build
rm -r *
cmake ..
make
./path_planning

