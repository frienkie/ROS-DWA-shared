#!/usr/bin/env python3

"""
Test script to verify ROS2 node imports and basic functionality
"""

import sys
import os

def test_imports():
    """Test if all required modules can be imported"""
    print("Testing imports...")
    
    try:
        import rclpy
        print("✓ rclpy imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import rclpy: {e}")
        return False
    
    try:
        from geometry_msgs.msg import Twist, Point
        print("✓ geometry_msgs imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import geometry_msgs: {e}")
        return False
    
    try:
        from nav_msgs.msg import Odometry
        print("✓ nav_msgs imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import nav_msgs: {e}")
        return False
    
    try:
        from sensor_msgs.msg import LaserScan
        print("✓ sensor_msgs imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import sensor_msgs: {e}")
        return False
    
    try:
        from tf_transformations import euler_from_quaternion
        print("✓ tf_transformations imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import tf_transformations: {e}")
        return False
    
    try:
        from std_msgs.msg import Float32
        print("✓ std_msgs imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import std_msgs: {e}")
        return False
    
    try:
        from visualization_msgs.msg import Marker
        print("✓ visualization_msgs imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import visualization_msgs: {e}")
        return False
    
    return True

def test_package_imports():
    """Test if our package modules can be imported"""
    print("\nTesting package imports...")
    
    try:
        from shared_core import dwa_ros2
        print("✓ dwa_ros2 module imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import dwa_ros2: {e}")
        return False
    
    try:
        from shared_core import dwa_remote_ros2
        print("✓ dwa_remote_ros2 module imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import dwa_remote_ros2: {e}")
        return False
    
    try:
        from shared_core import distancetime_ros2
        print("✓ distancetime_ros2 module imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import distancetime_ros2: {e}")
        return False
    
    try:
        from shared_core import distancetime0_ros2
        print("✓ distancetime0_ros2 module imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import distancetime0_ros2: {e}")
        return False
    
    return True

def test_node_classes():
    """Test if node classes can be instantiated"""
    print("\nTesting node classes...")
    
    try:
        import rclpy
        rclpy.init()
        
        from shared_core.dwa_ros2 import DWANode
        node = DWANode()
        print("✓ DWANode instantiated successfully")
        node.destroy_node()
        
        from shared_core.dwa_remote_ros2 import DWARemoteNode
        node = DWARemoteNode()
        print("✓ DWARemoteNode instantiated successfully")
        node.destroy_node()
        
        rclpy.shutdown()
        return True
        
    except Exception as e:
        print(f"✗ Failed to instantiate nodes: {e}")
        return False

def test_utility_functions():
    """Test if utility functions can be called"""
    print("\nTesting utility functions...")
    
    try:
        from shared_core import distancetime_ros2, distancetime0_ros2
        
        # Test marker creation
        markers1 = distancetime_ros2.markers
        markers2 = distancetime0_ros2.markers
        print("✓ Marker objects created successfully")
        
        # Test time functions
        time_val = distancetime_ros2.get_time(0)
        print("✓ Time functions work correctly")
        
        return True
        
    except Exception as e:
        print(f"✗ Failed to test utility functions: {e}")
        return False

def main():
    """Main test function"""
    print("ROS2 Node Test Script")
    print("=" * 50)
    
    # Test basic imports
    if not test_imports():
        print("\n❌ Basic imports failed!")
        return 1
    
    # Test package imports
    if not test_package_imports():
        print("\n❌ Package imports failed!")
        return 1
    
    # Test utility functions
    if not test_utility_functions():
        print("\n❌ Utility functions failed!")
        return 1
    
    # Test node instantiation
    if not test_node_classes():
        print("\n❌ Node instantiation failed!")
        return 1
    
    print("\n✅ All tests passed!")
    print("\nYou can now run the nodes with:")
    print("  ros2 run shared_core dwa_node --param 1")
    print("  ros2 run shared_core dwa_remote_node")
    print("\nNote: For dwa_remote_node, modify RECORD variable in the file to control recording.")
    
    return 0

if __name__ == '__main__':
    sys.exit(main()) 