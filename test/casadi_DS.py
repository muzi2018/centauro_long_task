import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

from casadi import *

x = MX.sym('x',2); # Two states

# Expression for ODE right-hand side
z = 1-x[1]**2
rhs = vertcat(z*x[0]-x[1],x[0])

ode = {}         # ODE declaration
ode['x']   = x   # states
ode['ode'] = rhs # right-hand side

# Construct a Function that integrates over 4s
F = integrator('F','cvodes',ode,0,4)

# Start from x=[0;1]
res = F(x0=[0,1])

print(res["xf"])

# Sensitivity wrt initial state
res = F(x0=x)
S = Function('S',[x],[jacobian(res["xf"],x)])
print(S([0,1]))

