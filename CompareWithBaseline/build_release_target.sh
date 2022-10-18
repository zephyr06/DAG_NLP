#!/bin/bash
cd ..
mkdir release
cd release
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --config Release -- -j 8
# make check -j8