import casadi as ca
import numpy as np
import pandas as pd

class MPC_Controller:
    def __init__(self, N=10, T=1.0, L=0.33):
        self.N = N       # Prediction horizon
        self.T = T       # Total time horizon
        self.dt = T / N  # Time step
        self.L = L       # Wheelbase

        self._setup_optimizer()     # Create optimization problem

    def _setup_optimizer(self):

        # State variables
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        v = ca.SX.sym('v')
        theta = ca.SX.sym('theta')

        # Control variables
        a = ca.SX.sym('a')   
        delta = ca.SX.sym('delta')

        # System dynamics (Bicycle model)
        xdot = v * ca.cos(theta)
        ydot = v * ca.sin(theta)
        vdot = a
        thetadot = v * ca.tan(delta) / self.L

        # Define system function
        self.f = ca.Function('f', [x, y, v, theta, a, delta], [xdot, ydot, vdot, thetadot])

        # Optimization variables
        self.opti = ca.Opti()
        self.U = self.opti.variable(2, self.N)  # [Acceleration, Steering]
        self.X = self.opti.variable(4, self.N + 1)  # [x, y, v, theta]
        

        # Cost function weights
        self.Q = np.diag([10, 10, 1, 1])  # Penalize position & velocity error
        self.R = np.diag([1, 1])  # Penalize control input

        # Reference trajectory
        self.X_ref = self.opti.parameter(4, self.N + 1)

        # The Cost function
        cost = 0
        for i in range(self.N):
            state_error = self.X[:, i] - self.X_ref[:, i]
            control_effort = self.U[:, i]
            cost += ca.mtimes([state_error.T, self.Q, state_error]) + ca.mtimes([control_effort.T, self.R, control_effort])
        
        self.opti.minimize(cost)

        # Constraints
        g = []
        for i in range(self.N):
            x_next = self.X[:, i] + self.dt * ca.vertcat(*self.f(self.X[0, i], self.X[1, i], self.X[2, i], self.X[3, i], self.U[0, i], self.U[1, i]))
            g.append(self.X[:, i+1] - x_next)

        self.opti.subject_to(ca.vertcat(*g) == 0)

        # Control limits
        self.opti.subject_to(self.opti.bounded(-2, self.U[0, :], 2))             # Acceleration limits
        self.opti.subject_to(self.opti.bounded(-0.4, self.U[1, :], 0.4))         # Steering limits

        # Solver
        self.opti.solver('ipopt')

    def solve_mpc(self, x, y, v, theta, X_ref):

        try:
            self.opti.set_value(self.X_ref, X_ref)
            sol = self.opti.solve()
            optimal_accel = sol.value(self.U[0, 0])
            optimal_steer = sol.value(self.U[1, 0])
            return optimal_accel, optimal_steer
        except RuntimeError:
            return 0.0, 0.0  

    def create_circular_trajectory(self, center_x, center_y, radius, start_theta, N):
        trajectory = np.zeros((4, N+1))  # (x, y, velocity, theta)
        target_velocity = 3.0  # m/s
        angular_velocity = target_velocity / radius  # Turn rate based on velocity
    
        for i in range(N+1):
            theta = start_theta + angular_velocity * i 
            trajectory[0, i] = center_x + radius * np.cos(theta) 
            trajectory[1, i] = center_y + radius * np.sin(theta)  
            trajectory[2, i] = target_velocity  
            trajectory[3, i] = theta  
    
        return trajectory
    
    def create_straight_line_trajectory(self, start_x, start_y, start_theta, end_x, end_y, N):
        trajectory = np.zeros((4, N+1))  # (x, y, velocity, theta)
    
        # Define the line
        delta_x = end_x - start_x
        delta_y = end_y - start_y
        distance = np.sqrt(delta_x**2 + delta_y**2)
    
        target_velocity = 1.0  # m/s
    
        for i in range(N+1):
            t = i / N
            trajectory[0, i] = start_x + t * delta_x 
            trajectory[1, i] = start_y + t * delta_y  
            trajectory[2, i] = target_velocity  
            trajectory[3, i] = start_theta
    
            return trajectory
        
    def followPurePursuit(self, current_state, lookahead_distance=0.5, csv_path="mpc_controller/mansour_3_out.csv"):
   
        # Load path with headers: x, y, v
        path = pd.read_csv(csv_path)

        if not {'x', 'y', 'v'}.issubset(path.columns):
            raise ValueError("CSV file must contain 'x', 'y', and 'v' columns.")

        x_path = path['x'].values
        y_path = path['y'].values
        v_path = path['v'].values

        x, y, v, theta = current_state

        # Find lookahead point closest to desired distance
        distances = np.hypot(x_path - x, y_path - y)
        lookahead_idx = np.argmin(np.abs(distances - lookahead_distance))

        # Extract N+1 points starting from lookahead index
        end_idx = min(lookahead_idx + self.N + 1, len(x_path))
        ref_x = x_path[lookahead_idx:end_idx]
        ref_y = y_path[lookahead_idx:end_idx]
        ref_v = v_path[lookahead_idx:end_idx]

        # Estimate theta between consecutive points
        thetas = np.arctan2(np.diff(ref_y, append=ref_y[-1]), np.diff(ref_x, append=ref_x[-1]))

        # Pad if needed to ensure shape (N+1,)
        while len(ref_x) < self.N + 1:
            ref_x = np.append(ref_x, ref_x[-1])
            ref_y = np.append(ref_y, ref_y[-1])
            ref_v = np.append(ref_v, 0.0)
            thetas = np.append(thetas, thetas[-1])

        # Stack to form X_ref of shape (4, N+1)
        X_ref = np.vstack((ref_x, ref_y, ref_v, thetas))

        # Solve MPC using generated reference
        return self.solve_mpc(x, y, v, theta, X_ref)
        


# Main function to test
#if __name__ == "__main__":
 #   print("Initializing MPC Controller...")
  #  mpc = MPC_Controller()
