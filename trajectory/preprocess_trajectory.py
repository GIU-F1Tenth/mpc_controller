import csv
import math
import os
import yaml
import argparse
import sys


def process_csv(input_path, output_path, wheelbase=0.33, max_steering=0.5, min_velocity=0.1):
    """Process trajectory CSV file and add steering angles"""
    try:
        with open(input_path, 'r') as infile:
            reader = csv.DictReader(infile)
            data = list(reader)

        processed = []
        N = len(data)

        for i in range(N):
            # Get current and next point for heading calculation
            i1 = i % N
            i2 = (i + 1) % N

            x1, y1 = float(data[i1]['x']), float(data[i1]['y'])
            x2, y2 = float(data[i2]['x']), float(data[i2]['y'])
            v = float(data[i1]['v'])

            # Ensure velocity is above minimum threshold for MPC stability
            v = max(v, min_velocity)

            # Calculate heading angle (theta) from consecutive points
            dx = x2 - x1
            dy = y2 - y1
            theta = math.atan2(dy, dx)

            # MPC controller will compute steering angles - no delta calculation here
            processed.append({
                'x': x1,
                'y': y1,
                'v': v,
                'theta': theta
            })

        with open(output_path, 'w', newline='') as outfile:
            fieldnames = ['x', 'y', 'v', 'theta']
            writer = csv.DictWriter(outfile, fieldnames=fieldnames)
            writer.writeheader()
            for row in processed:
                writer.writerow(row)

        print(f"[✓] Processed trajectory: {len(processed)} points")
        print(f"[✓] Output saved to: {output_path}")
        return True

    except Exception as e:
        print(f"[✗] Error processing trajectory: {str(e)}")
        return False


def load_ros2_params(config_path):
    """Load parameters from ROS2 params.yaml file"""
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Extract mpc_controller parameters
        if 'optimized_mpc_controller' in config:
            mpc_params = config.get('optimized_mpc_controller', {}).get('ros__parameters', {})
        elif 'mpc_controller' in config:
            mpc_params = config.get('mpc_controller', {}).get('ros__parameters', {})
        else:
            mpc_params = {}

        return {
            'optimal_trajectory_path': mpc_params.get('optimal_trajectory_path'),
            'reference_trajectory_path': mpc_params.get('reference_trajectory_path'),
            'wheelbase': mpc_params.get('wheelbase', 0.33),
            'max_steering_angle': mpc_params.get('max_steering_angle', 0.5),
            'min_speed': mpc_params.get('min_speed', 0.1),
            'enable_logging': mpc_params.get('enable_logging', True),
            'horizon_N': mpc_params.get('horizon_N', 10)
        }
    except Exception as e:
        print(f"[✗] Error loading ROS2 config: {e}")
        return {}


def find_config_file():
    """Automatically find config file in standard locations"""
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Search in common locations relative to script
    possible_configs = [
        os.path.join(script_dir, '..', 'config', 'params.yaml'),
        os.path.join(script_dir, 'config', 'params.yaml'),
        os.path.join(script_dir, '..', 'params.yaml'),
        os.path.join(script_dir, 'params.yaml'),
        'config/params.yaml',
        '../config/params.yaml',
        'params.yaml'
    ]

    for config_path in possible_configs:
        if os.path.exists(config_path):
            return os.path.abspath(config_path)

    return None


def preprocess_trajectory(
        input_path,
        output_path,
        config_path=None,
        wheelbase=None,
        max_steering=None,
        min_velocity=None):
    """
    Main function to be called from ROS2 node

    Args:
        input_path: Path to input CSV file
        output_path: Path to output CSV file
        config_path: Optional path to ROS2 params file
        wheelbase: Optional wheelbase value (overrides config)
        max_steering: Optional max steering angle (overrides config)
        min_velocity: Optional minimum velocity (overrides config)

    Returns:
        bool: True if successful, False otherwise
    """
    # Default values
    L_use = wheelbase or 0.33
    max_steer_use = max_steering or 0.5
    min_vel_use = min_velocity or 0.1

    # Load from config if provided
    if config_path and os.path.exists(config_path):
        params = load_ros2_params(config_path)
        if not wheelbase:
            L_use = params.get('wheelbase', 0.33)
        if not max_steering:
            max_steer_use = params.get('max_steering_angle', 0.5)
        if not min_velocity:
            min_vel_use = params.get('min_speed', 0.1)

    # Create output directory if it doesn't exist
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    return process_csv(input_path, output_path, L_use, max_steer_use, min_vel_use)


def create_sample_trajectory(output_dir="trajectory"):
    """Create a sample trajectory CSV for testing"""
    os.makedirs(output_dir, exist_ok=True)
    sample_path = os.path.join(output_dir, "optimal_trajectory.csv")

    # Generate a simple figure-8 trajectory
    points = []
    num_points = 100

    for i in range(num_points):
        t = 2 * math.pi * i / num_points
        # Figure-8 parametric equations
        x = 5 * math.sin(t)
        y = 2.5 * math.sin(2 * t)
        # Ensure velocity is always positive and above minimum threshold
        v = 1.5 + 0.8 * math.cos(t)  # Range: [0.7, 2.3] m/s - safe for MPC

        points.append({'x': x, 'y': y, 'v': v})

    with open(sample_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['x', 'y', 'v'])
        writer.writeheader()
        writer.writerows(points)

    print(f"[✓] Created sample trajectory: {sample_path}")
    return sample_path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Preprocess trajectory CSV files for MPC controller')
    parser.add_argument('-i', '--input', help='Input CSV file path')
    parser.add_argument('-o', '--output', help='Output CSV file path')
    parser.add_argument('--create-sample', action='store_true', help='Create sample trajectory for testing')

    args = parser.parse_args()

    # Handle sample creation
    if args.create_sample:
        sample_path = create_sample_trajectory()
        print(f"[✓] Sample trajectory created: {sample_path}")
        sys.exit(0)

    # Handle direct command line arguments
    if args.input and args.output:
        # Auto-find config file
        config_path = find_config_file()
        if config_path:
            print(f"[✓] Found config file: {config_path}")

        success = preprocess_trajectory(args.input, args.output, config_path)
        sys.exit(0 if success else 1)

    # Auto-find config file and process
    config_path = find_config_file()
    if config_path:
        print(f"[✓] Found config file: {config_path}")
        params = load_ros2_params(config_path)
        input_path = params.get('optimal_trajectory_path')
        output_path = params.get('reference_trajectory_path')

        if input_path and output_path:
            success = preprocess_trajectory(input_path, output_path, config_path)
            if success:
                print("[✓] Trajectory preprocessing completed successfully")
            else:
                print("[✗] Trajectory preprocessing failed")
        else:
            print("[✗] Missing trajectory paths in config file")
    else:
        print("[!] No config file found.")
        print("Expected config file at: ../config/params.yaml")
        print("Usage examples:")
        print("  python3 preprocess_trajectory.py")
        print("  python3 preprocess_trajectory.py --create-sample")
        print("  python3 preprocess_trajectory.py -i input.csv -o output.csv")
