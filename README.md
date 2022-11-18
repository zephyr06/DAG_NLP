# Dependencies:
- GTSAM
- boost
- OpenCV
- Python
    - Graphviz (conda install python-graphviz)
    - pydot (conda install -c rmg pydot)
- GCC V8.1+ (`sudo apt install gcc-8 g++-8`)
- [CppUnitLite](https://github.com/Zephyr06/CppUnitLite) (If you want to run test files)

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
