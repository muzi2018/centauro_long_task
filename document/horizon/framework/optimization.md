# Optimization

## set parameter

the horizon period : $t_f$

the number of nodes : $n_s$

the time segment: $dt = t_f / n_s$

## set transcription

set name

set the type of integrator

## create horizon problem

how much nodes

```python
prb = problem.Problem(ns)
```

what are the variables

```python
# STATE variables
q = prb.createStateVariable("q", nq)
qdot = prb.createStateVariable("qdot", nv)
# CONTROL variables
qddot = prb.createInputVariable("qddot", nv)
# Creates double integrator
xdot = utils.double_integrator(q, qdot, qddot) # =[v a]
```

set dynamics of the system and the segment time

```python
# Set dynamics of the system and the relative dt
prb.setDynamics(xdot)
prb.setDt(dt)
```

set bounds and initial guess

```python
# joint limits + initial pos
q_min = [-0.5, -2.*np.pi]
q_max = [0.5, 2.*np.pi]
q_init = [0., 0.]
# velocity limits + initial vel
qdot_lims = np.array([100., 100.])
qdot_init = [0., 0.]
# acceleration limits
qddot_lims = np.array([1000., 1000.])
qddot_init = [0., 0.]

# Set bounds
q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, nodes=0)
qdot.setBounds(-qdot_lims, qdot_lims)
qdot.setBounds(qdot_init, qdot_init, nodes=0)
qddot.setBounds(-qddot_lims, qddot_lims)
```

```python
# Set initial guess
q.setInitialGuess(q_init)
qdot.setInitialGuess(qdot_init)
qddot.setInitialGuess(qddot_init)
```

set transcription method

```python
th = Transcriptor.make_method(transcription_method, prb, opts=transcription_opts)
```

**set constraints**

```python
# Set dynamic feasibility:
# the cart can apply a torque, the joint connecting the cart to the pendulum is UNACTUATED
# the torques are computed using the inverse dynamics, as the input of the problem is the cart acceleration
tau_lims = np.array([1000., 0.])
tau = kin_dyn.InverseDynamics(kindyn).call(q, qdot, qddot)
iv = prb.createIntermediateConstraint("dynamic_feasibility", tau, bounds=dict(lb=-tau_lims, ub=tau_lims))

# at the last node, the pendulum is upright
prb.createFinalConstraint("up", q[1] - np.pi)
# at the last node, the system velocity is zero
prb.createFinalConstraint("final_qdot", qdot)
```

**set cost**

```python
# minimize the acceleration of system (regularization of the input)
prb.createIntermediateCost("qddot", cs.sumsqr(qddot))
```

**build problem**

```python
# ==================== BUILD PROBLEM ===============================
# the solver class accept different solvers, such as 'ipopt', 'ilqr', 'gnsqp'.
# Different solver are useful (and feasible) in different situations.
solv = solver.Solver.make_solver('ipopt', prb, opts={'ipopt.tol': 1e-4,'ipopt.max_iter': 2000})

# ==================== SOLVE PROBLEM ===============================
solv.solve()
```
