# Dependencies:
- [GTSAM](https://github.com/borglab/gtsam)
    - Please install the old version GTSAM (commit `9902ccc0a`) to avoid API issues
    - Add `set (GTSAM_USE_SYSTEM_EIGEN 1)` to `CMakeLists.txt` in case of Eigen library issues
- boost
- OpenCV, C++
- Python
    - Graphviz (conda install python-graphviz)
    - pydot (conda install -c rmg pydot)
- GCC V8.1+ (`sudo apt install gcc-8 g++-8`)
- [CppUnitLite](https://github.com/Zephyr06/CppUnitLite) 
- yaml-cpp (sudo apt install libyaml-cpp-dev)
- Cplex, C++

# Run all the tests
```
cd build
make check.UnitTests -j2
```
# Generate random task sets
```
make GenerateTaskSet
 ./tests/GenerateTaskSet --N 10 --taskSetNumber 10 --SF_ForkNum 2 --numCauseEffectChain 2
```

# Run the scripts for large-scale experiments
```
cd build
make check.scripts -j2
```
`