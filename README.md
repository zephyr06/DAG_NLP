# Dependencies:
- GTSAM
- boost
- OpenCV
- Python
    - Graphviz (conda install python-graphviz)
    - pydot (conda install -c rmg pydot)
- GCC V8.1+ (`sudo apt install gcc-8 g++-8`)
- [CppUnitLite](https://github.com/Zephyr06/CppUnitLite) (If you want to run test files)

# TODO:
1. Gradient vanish problem
3. Event chain analysis
- ErrorDimension = final job's instances
- direct modeling: back-ward propogation to get event chain's response time
- helper modeling: derive necessary condition for each instance, and add barrier function to them

# test todo:

- * interval cover occurs at later stages *
---> solution: do not initialize with big overlap error

# Big TODO:
- Event chain factor: analysis, approximation and implementation
- Gradient vanish:
- find out why does UnitOptimization take so much time
- different results when using debug vs release built option
- better support for float-point execution time, especially how to generate initial solution
- I doubt the inner iterations are using dense Hessian, but I'm not sure

# IDEA
- DIVIDE the solution space appropriately, like cliques in bayes tree
