#!/bin/sh
cmake -Bbuild -DCMAKE_BUILD_TYPE=Release
cd build
make
./cpsc587_a5_hh

