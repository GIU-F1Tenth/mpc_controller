import pandas as pd
import numpy as np
from mpc_controller.mpc_controller import MPC_Controller  
import math

def estimate_theta_from_xy(x, y):
    dx = np.diff(x)
    dy = np.diff(y)
    theta = np.arctan2(dy, dx)
    # Repeat last theta to keep size consistent
    theta = np.append(theta, theta[-1])
    return theta

def simulate_forward(state, a, delta, dt, L):
    x, y, v, theta = state
    x_dot = v * np.cos(theta)
    y_dot = v * np.sin(theta)
    v_dot = a
    theta_dot = v * np.tan(delta) / L
    x += x_dot * dt
    y += y_dot * dt
    v += v_dot * dt
    theta += theta_dot * dt
    return [x, y, v, theta]

def main():
    input_csv = "/home/mohammedazab/GIU_F1Tenth/mpc_controller/mpc_controller/input_trajectory.csv"
    output_csv = 'optimized_trajectory.csv'
    
    # Load input
    df = pd.read_csv(input_csv)
    x = df['x'].to_numpy()
    y = df['y'].to_numpy()
    v = df['v'].to_numpy()

    # Estimate theta from x, y
    theta = estimate_theta_from_xy(x, y)

    # Form full reference trajectory
    trajectory = np.vstack([x, y, v, theta])  # shape (4, N+1)
    N = min(10, trajectory.shape[1] - 1)  # Adjust N if needed

    # Initialize MPC
    mpc = MPC_Controller(N=N, T=1.0, L=0.33)

    optimized_states = []
    optimized_controls = []

    # Start from the first state
    state = trajectory[:, 0]

    for t in range(trajectory.shape[1] - N - 1):
        X_ref = trajectory[:, t:t+N+1]

        a, delta = mpc.solve_mpc(*state, X_ref)
        optimized_controls.append([a, delta])
        optimized_states.append(state)

        # Simulate one step forward
        state = simulate_forward(state, a, delta, mpc.dt, mpc.L)

    # Add final state
    optimized_states.append(state)

    # Save to CSV
    optimized_states = np.array(optimized_states)
    optimized_controls = np.array(optimized_controls)
    final_len = len(optimized_states)

    result_df = pd.DataFrame({
        'x': optimized_states[:, 0],
        'y': optimized_states[:, 1],
        'v': optimized_states[:, 2],
        'theta': optimized_states[:, 3],
        'a': np.append(optimized_controls[:, 0], 0),
        'delta': np.append(optimized_controls[:, 1], 0)
    })

    result_df.to_csv(output_csv, index=False)
    print(f"✅ Optimized trajectory saved to: {output_csv}")

if __name__ == '__main__':
    main()
