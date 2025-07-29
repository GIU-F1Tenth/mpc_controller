#!/usr/bin/env python3
"""
Configuration Comparison Script

This script compares configuration values between config.py and params.yaml files
to ensure they are in sync.
"""

import yaml
import os
import sys
from typing import Dict, Any, List, Tuple


def load_python_config(file_path: str) -> Dict[str, Any]:
    """Load configuration from Python file"""
    config = {}
    
    # Read the Python file and extract variable assignments
    with open(file_path, 'r') as f:
        lines = f.readlines()
    
    for line in lines:
        line = line.strip()
        if line and not line.startswith('#') and '=' in line:
            # Parse variable assignment
            parts = line.split('=', 1)
            if len(parts) == 2:
                key = parts[0].strip()
                value = parts[1].strip()
                
                # Remove trailing comments
                if '#' in value:
                    value = value.split('#')[0].strip()
                
                # Parse value
                try:
                    # Handle string values
                    if value.startswith('"') and value.endswith('"'):
                        config[key] = value[1:-1]
                    elif value.startswith("'") and value.endswith("'"):
                        config[key] = value[1:-1]
                    # Handle boolean values
                    elif value.lower() == 'true':
                        config[key] = True
                    elif value.lower() == 'false':
                        config[key] = False
                    # Handle numeric values
                    else:
                        try:
                            if '.' in value:
                                config[key] = float(value)
                            else:
                                config[key] = int(value)
                        except ValueError:
                            # If it's not a number, keep as string
                            config[key] = value
                except:
                    config[key] = value
    
    return config


def load_yaml_config(file_path: str) -> Dict[str, Any]:
    """Load configuration from YAML file"""
    with open(file_path, 'r') as f:
        yaml_data = yaml.safe_load(f)
    
    # Extract parameters from the nested structure
    config = {}
    
    # Extract from optimized_mpc_controller section
    if 'optimized_mpc_controller' in yaml_data:
        mpc_params = yaml_data['optimized_mpc_controller'].get('ros__parameters', {})
        
        # Flatten cost_function_weights
        if 'cost_function_weights' in mpc_params:
            weights = mpc_params['cost_function_weights']
            for key, value in weights.items():
                config[key] = value
            del mpc_params['cost_function_weights']
        
        # Flatten hard_constraints
        if 'hard_constraints' in mpc_params:
            constraints = mpc_params['hard_constraints']
            for key, value in constraints.items():
                config[f'hard_{key}'] = value
            del mpc_params['hard_constraints']
        
        # Add remaining parameters
        config.update(mpc_params)
    
    return config


def compare_configs(py_config: Dict[str, Any], yaml_config: Dict[str, Any]) -> Tuple[List[str], List[str], List[str]]:
    """Compare two configuration dictionaries"""
    matches = []
    mismatches = []
    missing = []
    
    # Get all unique keys from both configs
    all_keys = set(py_config.keys()) | set(yaml_config.keys())
    
    for key in sorted(all_keys):
        py_value = py_config.get(key, '<MISSING>')
        yaml_value = yaml_config.get(key, '<MISSING>')
        
        if key not in py_config:
            missing.append(f"'{key}' missing in config.py (YAML value: {yaml_value})")
        elif key not in yaml_config:
            missing.append(f"'{key}' missing in params.yaml (Python value: {py_value})")
        elif py_value == yaml_value:
            matches.append(f"'{key}': {py_value}")
        else:
            mismatches.append(f"'{key}': Python={py_value}, YAML={yaml_value}")
    
    return matches, mismatches, missing


def print_results(matches: List[str], mismatches: List[str], missing: List[str]):
    """Print comparison results"""
    print("=" * 80)
    print("CONFIGURATION COMPARISON RESULTS")
    print("=" * 80)
    
    print(f"\n✅ MATCHING PARAMETERS ({len(matches)}):")
    print("-" * 40)
    if matches:
        for match in matches:
            print(f"  {match}")
    else:
        print("  No matching parameters found")
    
    print(f"\n❌ MISMATCHED PARAMETERS ({len(mismatches)}):")
    print("-" * 40)
    if mismatches:
        for mismatch in mismatches:
            print(f"  {mismatch}")
    else:
        print("  No mismatched parameters found")
    
    print(f"\n⚠️  MISSING PARAMETERS ({len(missing)}):")
    print("-" * 40)
    if missing:
        for miss in missing:
            print(f"  {miss}")
    else:
        print("  No missing parameters found")
    
    print("\n" + "=" * 80)
    
    # Summary
    total_params = len(matches) + len(mismatches) + len(missing)
    if total_params > 0:
        match_percentage = (len(matches) / total_params) * 100
        print(f"SUMMARY: {len(matches)}/{total_params} parameters match ({match_percentage:.1f}%)")
    else:
        print("SUMMARY: No parameters found to compare")
    
    if len(mismatches) == 0 and len(missing) == 0:
        print("🎉 ALL CONFIGURATIONS ARE IN SYNC!")
        return True
    else:
        print("⚠️  CONFIGURATIONS ARE OUT OF SYNC!")
        return False


def main():
    """Main function"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # File paths
    py_config_path = os.path.join(script_dir, 'config.py')
    yaml_config_path = os.path.join(script_dir, 'params.yaml')
    
    # Check if files exist
    if not os.path.exists(py_config_path):
        print(f"Error: {py_config_path} not found!")
        sys.exit(1)
    
    if not os.path.exists(yaml_config_path):
        print(f"Error: {yaml_config_path} not found!")
        sys.exit(1)
    
    print(f"Comparing:")
    print(f"  Python config: {py_config_path}")
    print(f"  YAML config:   {yaml_config_path}")
    
    try:
        # Load configurations
        py_config = load_python_config(py_config_path)
        yaml_config = load_yaml_config(yaml_config_path)
        
        print(f"\nLoaded {len(py_config)} parameters from Python config")
        print(f"Loaded {len(yaml_config)} parameters from YAML config")
        
        # Compare configurations
        matches, mismatches, missing = compare_configs(py_config, yaml_config)
        
        # Print results
        in_sync = print_results(matches, mismatches, missing)
        
        # Exit with appropriate code
        sys.exit(0 if in_sync else 1)
        
    except Exception as e:
        print(f"Error during comparison: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
