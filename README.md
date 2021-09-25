#TODO:
1. DBF Jacobian all 0, but error is not 0
- the interval that causes error has been eliminated
--> When performing elimination, we should check whether some variables that depend on it
--> probably, current recover can't handle multiple dependency relationship well
2. some some error from DBF simple won't go away(v18, delta 1e-1)


test todo:

What's elimination condition?
advanced initialization
figure more about termination condition
- * interval cover occurs at later stages *
---> solution: do not initialize with big overlap error

For 22 test case, we hope to make the optimizer 'behave' based on DBF Jacobian matrix
Q: why doesn't DBF Jacobian guide the optimization?

# Recent results:
RM-NLP: 179 failed
RM-SA: 228 failed
RM-Init: 228 failed


# IDEA
- DIVIDE the solution space appropriately, like cliques in bayes tree