# Dependencies:
- [GTSAM](https://github.com/borglab/gtsam)
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

# Run the scripts for large-scale experiments
```
cd build
make check.scripts -j2
```
`