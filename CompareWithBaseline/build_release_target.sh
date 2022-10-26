#!/bin/bash
cd ..
mkdir release
cd release
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake -DCMAKE_BUILD_TYPE=Release ..
<<<<<<< HEAD
cmake --build . --config Release -- -j 4
# make check -j4
=======
cmake --build . --config Release -- -j 6
# make check -j6
>>>>>>> 23b437c294a5fdb2af503c4240e21df31090fe54
