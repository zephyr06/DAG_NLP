#!/bin/bash
cd ..
mkdir release
cd release
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
make check -j8