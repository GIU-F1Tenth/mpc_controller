import numpy as np

class MPC:
    def __init__(self):
        self.dt = 0.1
        self.L = 0.3  # wheelbase
        self.horizon = 5

    def solve(self, state, trajectory, idx):
        target_idx = min(idx + 1, len(trajectory["x"]) - 1)
        dx = trajectory["x"][target_idx] - state[0]
        dy = trajectory["y"][target_idx] - state[1]
        angle = np.arctan2(dy, dx) - state[2]
        steering = np.clip(angle, -0.8, 0.8)
        speed = trajectory["speed"][target_idx]
        return [steering, speed]

