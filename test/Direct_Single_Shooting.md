# dynamics

$$
x=(x_0, x_1)
$$

$$
\dot{x_0}=(1-x_1^2)x_0 - x_1 + 2\text{tanh}(p)\\
\dot{x_1}=x_0
$$

$$
\dot x = f(x,p)
$$

## Discrete

$$
p \rightarrow \text{discrete} \rightarrow p[0] \cdots p[N]
$$

$$
x \rightarrow \text{discrete} \rightarrow x[0] \cdots x[N]
$$

$$
\dot x[0] =f(x[0], p[0]) \\
\vdots \\
\text{integral} \\
\vdots \\ 
\dot x[N] =f(x[N], p[N])
$$

## Replace parameter p

$$
u[0] \cdots u[1] \rightarrow  p[0] \cdots p[1]
$$

Then become Direct_Single_Shooting
