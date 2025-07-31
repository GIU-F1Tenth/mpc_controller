#!/usr/bin/env python3
"""
MPC Parameter Update Test Script

Quick test script to verify that the MPC parameter update mechanism works correctly.
This script can be used to test parameter updates without the full GUI.

Usage:
    python3 test_parameter_update.py
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import time


class ParameterTester(Node):
    def __init__(self):
        super().__init__('parameter_tester')
        self.get_logger().info("Parameter Tester Node started")
    
    def test_parameter_update(self):
        """Test updating MPC parameters"""
        
        # Check if MPC node is available
        node_names = self.get_node_names()
        if 'optimized_mpc_node' not in node_names:
            self.get_logger().error("MPC node 'optimized_mpc_node' not found!")
            self.get_logger().info(f"Available nodes: {node_names}")
            return False
        
        self.get_logger().info("✅ MPC node found!")
        
        # Create parameter client
        param_client = self.create_client(
            rclpy.parameter.SetParameters,
            '/optimized_mpc_node/set_parameters'
        )
        
        # Wait for service
        if not param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ Parameter service not available!")
            return False
        
        self.get_logger().info("✅ Parameter service available!")
        
        # Test parameter updates
        test_parameters = [
            Parameter('max_speed', Parameter.Type.DOUBLE, 5.0),
            Parameter('max_steering_angle', Parameter.Type.DOUBLE, 0.4),
            Parameter('cost_function_weights.steering_weight', Parameter.Type.DOUBLE, 0.15),
            Parameter('enable_safety_checks', Parameter.Type.BOOL, True),
            Parameter('mpc_type', Parameter.Type.STRING, 'kinematic'),
        ]
        
        # Send test parameters
        request = rclpy.parameter.SetParameters.Request()
        request.parameters = test_parameters
        
        self.get_logger().info("🔄 Sending test parameters...")
        
        future = param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info("✅ Parameter update successful!")
            
            # Check results
            for result in response.results:
                status = "SUCCESS" if result.successful else "FAILED"
                self.get_logger().info(f"   Parameter update: {status}")
                if result.reason:
                    self.get_logger().info(f"   Reason: {result.reason}")
            
            return True
        else:
            self.get_logger().error("❌ Parameter update failed!")
            return False


def main():
    """Main test function"""
    print("🏎️  F1TENTH MPC Parameter Update Test")
    print("=====================================")
    
    rclpy.init()
    
    try:
        tester = ParameterTester()
        
        # Run the test
        success = tester.test_parameter_update()
        
        if success:
            print("\n✅ Test completed successfully!")
            print("💡 The MPC parameter update mechanism is working correctly.")
            print("   You can now use the tuning GUI with confidence.")
        else:
            print("\n❌ Test failed!")
            print("💡 Troubleshooting tips:")
            print("   1. Make sure the MPC controller is running:")
            print("      ros2 launch mpc_controller mpc_controller.launch.py")
            print("   2. Check that ROS2 is properly sourced")
            print("   3. Verify the workspace is built: colcon build --packages-select mpc_controller")
        
        tester.destroy_node()
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
