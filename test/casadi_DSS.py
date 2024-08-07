from casadi import *
from matplotlib.pyplot import plot, show

x = MX.sym('x',2)  # Two states
p = MX.sym('p')    # Free parameter

# Expression for ODE right-hand side
z = 1-x[1]**2
rhs = vertcat(z*x[0]-x[1]+2*tanh(p),x[0])

# ODE declaration with free parameter
ode = {'x':x,'p':p,'ode':rhs}

# Construct a Function that integrates over 1s
F = integrator('F','cvodes',ode,0,1)

# Control vector
u = MX.sym('u',4,1)

x = [0,1]  # Initial state
for k in range(4):
  # Integrate 1s forward in time:
  # call integrator symbolically
  res = F(x0=x,p=u[k])   # the u is variables, x is dependent on u
  x = res['xf']



# NLP declaration
nlp = {'x':u,'f':dot(u,u),'g':x}

# Solve using IPOPT
solver = nlpsol('solver','ipopt',nlp)
initial_guess_u = np.zeros((4, 1))

res = solver(x0=initial_guess_u,lbg=0,ubg=0)

print(res['x'])
print(res['f'])
# exit()
plot(res['x'])
show()

