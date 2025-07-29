#!/usr/bin/env python3
"""
Quick Config Check

A simple script to quickly check if config.py and params.yaml are synchronized.
Useful for CI/CD pipelines and quick validation.
"""

import sys
import os

# Add the current directory to the path to import config_sync
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from config_sync import ConfigComparator
except ImportError:
    print("Error: config_sync module not found!")
    sys.exit(1)


def quick_check():
    """Perform a quick synchronization check"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    py_config_path = os.path.join(script_dir, 'config.py')
    yaml_config_path = os.path.join(script_dir, 'params.yaml')
    
    if not os.path.exists(py_config_path):
        print(f"❌ config.py not found!")
        return False
    
    if not os.path.exists(yaml_config_path):
        print(f"❌ params.yaml not found!")
        return False
    
    try:
        comparator = ConfigComparator(py_config_path, yaml_config_path)
        matches, mismatches, missing = comparator.compare()
        
        total = len(matches) + len(mismatches) + len(missing)
        if total == 0:
            print("⚠️  No parameters found!")
            return False
        
        sync_percentage = (len(matches) / total) * 100
        
        if len(mismatches) == 0 and len(missing) == 0:
            print(f"✅ Config files are synchronized ({sync_percentage:.0f}%)")
            return True
        else:
            print(f"❌ Config files are NOT synchronized ({sync_percentage:.0f}%)")
            print(f"   - {len(mismatches)} mismatched parameters")
            print(f"   - {len(missing)} missing parameters")
            return False
    
    except Exception as e:
        print(f"❌ Error during check: {e}")
        return False


if __name__ == "__main__":
    success = quick_check()
    sys.exit(0 if success else 1)
