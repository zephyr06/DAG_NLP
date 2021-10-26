# Dependencies:
- GTSAM
- boost
- Python
    - Graphviz
    - pydot (conda install -c rmg pydot)

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

# IDEA
- DIVIDE the solution space appropriately, like cliques in bayes tree