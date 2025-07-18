"""
Trajectory optimization using Model Predictive Control.

This script optimizes a given input trajectory using the MPC controller
for the F1Tenth vehicle.
"""

import logging
import os
from pathlib import Path
from typing import Tuple
import pandas as pd
import numpy as np
from mpc_controller.MPC_Controller import MPCController

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TrajectoryOptimizer:
    """Optimizes vehicle trajectories using Model Predictive Control."""

    def __init__(self, wheelbase: float = 0.33):
        """
        Initialize the trajectory optimizer.

        Args:
            wheelbase: Vehicle wheelbase in meters
        """
        self.wheelbase = wheelbase
        self.mpc = None

    def estimate_heading_from_positions(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """
        Estimate heading angles from position data.

        Args:
            x: Array of x coordinates
            y: Array of y coordinates

        Returns:
            Array of heading angles in radians
        """
        if len(x) != len(y) or len(x) < 2:
            raise ValueError(
                "x and y must have same length and at least 2 points")

        dx = np.diff(x)
        dy = np.diff(y)
        theta = np.arctan2(dy, dx)

        # Extend last heading to maintain array size
        theta = np.append(theta, theta[-1])

        return theta

    def simulate_vehicle_step(
        self,
        state: np.ndarray,
        acceleration: float,
        steering_angle: float,
        dt: float
    ) -> np.ndarray:
        """
        Simulate one step of vehicle dynamics.

        Args:
            state: Current state [x, y, v, theta]
            acceleration: Acceleration command
            steering_angle: Steering angle command
            dt: Time step

        Returns:
            Next state [x, y, v, theta]
        """
        x, y, v, theta = state

        # Kinematic bicycle model
        x_dot = v * np.cos(theta)
        y_dot = v * np.sin(theta)
        v_dot = acceleration
        theta_dot = v * np.tan(steering_angle) / self.wheelbase

        # Euler integration
        x_next = x + x_dot * dt
        y_next = y + y_dot * dt
        v_next = max(0.0, v + v_dot * dt)  # Ensure non-negative velocity
        theta_next = theta + theta_dot * dt

        return np.array([x_next, y_next, v_next, theta_next])

    def optimize_trajectory(
        self,
        input_trajectory: np.ndarray,
        horizon: int = 10,
        total_time: float = 1.0
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Optimize a trajectory using MPC.

        Args:
            input_trajectory: Input trajectory as [x, y, v, theta] (4 x N)
            horizon: MPC prediction horizon
            total_time: MPC time horizon

        Returns:
            Tuple of (optimized_states, optimized_controls)
        """
        if input_trajectory.shape[0] != 4:
            raise ValueError("Input trajectory must have shape (4, N)")

        N_points = input_trajectory.shape[1]
        effective_horizon = min(horizon, N_points - 1)

        # Initialize MPC controller
        self.mpc = MPCController(
            N=effective_horizon, T=total_time, L=self.wheelbase)

        optimized_states = []
        optimized_controls = []

        # Start from initial state
        current_state = input_trajectory[:, 0]

        # Optimize trajectory
        for t in range(N_points - effective_horizon):
            # Extract reference trajectory segment
            X_ref = input_trajectory[:, t:t + effective_horizon + 1]

            # Solve MPC
            acceleration, steering = self.mpc.solve_mpc(
                current_state[0], current_state[1],
                current_state[2], current_state[3],
                X_ref
            )

            # Store results
            optimized_states.append(current_state.copy())
            optimized_controls.append([acceleration, steering])

            # Simulate next state
            current_state = self.simulate_vehicle_step(
                current_state, acceleration, steering, self.mpc.dt
            )

            if t % 10 == 0:
                logger.debug(
                    f"Optimized step {t}/{N_points - effective_horizon}")

        # Add final state
        optimized_states.append(current_state)

        return np.array(optimized_states).T, np.array(optimized_controls).T

    def load_trajectory_from_csv(self, filepath: str) -> np.ndarray:
        """
        Load trajectory from CSV file.

        Args:
            filepath: Path to CSV file

        Returns:
            Trajectory array (4 x N)
        """
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Trajectory file not found: {filepath}")

        try:
            df = pd.read_csv(filepath)

            # Check required columns
            required_cols = ['x', 'y', 'v']
            missing_cols = [
                col for col in required_cols if col not in df.columns]
            if missing_cols:
                raise ValueError(f"Missing required columns: {missing_cols}")

            x = df['x'].to_numpy()
            y = df['y'].to_numpy()
            v = df['v'].to_numpy()

            # Estimate or load heading
            if 'theta' in df.columns:
                theta = df['theta'].to_numpy()
            else:
                logger.info(
                    "Heading not found in CSV, estimating from positions")
                theta = self.estimate_heading_from_positions(x, y)

            trajectory = np.vstack([x, y, v, theta])
            logger.info(f"Loaded trajectory with {trajectory.shape[1]} points")

            return trajectory

        except Exception as e:
            raise RuntimeError(
                f"Failed to load trajectory from {filepath}: {e}")

    def save_trajectory_to_csv(
        self,
        states: np.ndarray,
        controls: np.ndarray,
        filepath: str
    ) -> None:
        """
        Save optimized trajectory to CSV file.

        Args:
            states: State trajectory (4 x N)
            controls: Control trajectory (2 x N-1)
            filepath: Output file path
        """
        try:
            # Pad controls to match states length
            controls_padded = np.column_stack([
                controls.T,
                np.zeros(controls.shape[0])
            ])

            result_df = pd.DataFrame({
                'x': states[0, :],
                'y': states[1, :],
                'v': states[2, :],
                'theta': states[3, :],
                'acceleration': controls_padded[:, 0],
                'steering_angle': controls_padded[:, 1]
            })

            result_df.to_csv(filepath, index=False)
            logger.info(f"Optimized trajectory saved to: {filepath}")

        except Exception as e:
            raise RuntimeError(f"Failed to save trajectory to {filepath}: {e}")


def main():
    """Main function to run trajectory optimization."""
    # Configure paths
    script_dir = Path(__file__).parent
    input_file = script_dir / "input_trajectory.csv"
    output_file = script_dir.parent / "optimized_trajectory.csv"

    try:
        # Initialize optimizer
        optimizer = TrajectoryOptimizer(wheelbase=0.33)

        # Load input trajectory
        logger.info(f"Loading trajectory from: {input_file}")
        input_trajectory = optimizer.load_trajectory_from_csv(str(input_file))

        # Optimize trajectory
        logger.info("Starting trajectory optimization...")
        optimized_states, optimized_controls = optimizer.optimize_trajectory(
            input_trajectory, horizon=10, total_time=1.0
        )

        # Save results
        optimizer.save_trajectory_to_csv(
            optimized_states, optimized_controls, str(output_file)
        )

        logger.info("Trajectory optimization completed successfully!")

    except Exception as e:
        logger.error(f"Trajectory optimization failed: {e}")
        raise


if __name__ == '__main__':
    main()
