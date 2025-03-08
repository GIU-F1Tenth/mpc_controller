import casadi as ca
import numpy as np

# Define MPC parameters
T = 1.0  # Time horizon (seconds)
N = 10   # Number of prediction steps
dt = T / N  # Time step per iteration

# Vehicle parameters
L = 0.33  # Wheelbase

# Define state variables
x = ca.SX.sym('x')
y = ca.SX.sym('y')
v = ca.SX.sym('v')
theta = ca.SX.sym('theta')

# Define control variables
a = ca.SX.sym('a')   # Acceleration
delta = ca.SX.sym('delta')  # Steering angle

# Define system dynamics using the Bicycle Model
xdot = v * ca.cos(theta)
ydot = v * ca.sin(theta)
vdot = a
thetadot = v * ca.tan(delta) / L

# Define state-space function
f = ca.Function('f', [x, y, v, theta, a, delta], [xdot, ydot, vdot, thetadot])

# Define optimization variables
U = ca.MX.sym('U', 2, N)  # Control inputs (a, delta) for N steps
X = ca.MX.sym('X', 4, N+1)  # States for N+1 steps

# Cost function weights
Q = np.diag([10, 10, 1, 1])  # Weights for (x, y, v, theta)
R = np.diag([1, 1])          # Weights for (acceleration, steering)

# Reference trajectory
X_ref = ca.MX.sym('X_ref', 4, N+1)

cost = 0
for i in range(N):
    state_error = X[:, i] - X_ref[:, i]
    control_effort = U[:, i]
    cost += ca.mtimes([state_error.T, Q, state_error]) + ca.mtimes([control_effort.T, R, control_effort])

