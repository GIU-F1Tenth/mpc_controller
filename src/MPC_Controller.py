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
