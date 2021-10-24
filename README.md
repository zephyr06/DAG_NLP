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
4. DBF Jacobian factor, do we use single dimension or M-dimension?
5. When transforming from numerical Jacobian to analytic Jacobian, how to deal with 'discrete part'?

# test todo:

- * interval cover occurs at later stages *
---> solution: do not initialize with big overlap error
- Test v35 for DBF zero error

# Big TODO:
- switch most factors to analytic Jacobian, and remove large '0' blocks if possible
- Event chain factor: analysis, approximation and implementation
- Gradient vanish:


# IDEA
- DIVIDE the solution space appropriately, like cliques in bayes tree