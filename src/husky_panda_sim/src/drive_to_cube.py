#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class HuskyDriver(Node):
    def __init__(self):
        super().__init__('husky_driver')
        
        # Topic determined by diff_drive_controller configuration
        # If use_stamped_vel is False, it often looks for 'cmd_vel_unstamped'
        self.publisher_ = self.create_publisher(Twist, '/husky_velocity_controller/cmd_vel_unstamped', 10)
        
    def move_forward(self, distance, speed=0.5):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        
        duration = distance / speed
        self.get_logger().info(f"Driving forward {distance}m at {speed}m/s for {duration}s...")
        
        # Simple open-loop control (Time based)
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05) # 20Hz
            
        # Stop the robot
        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info("Target Reached. Stopping.")

def main(args=None):
    rclpy.init(args=args)
    node = HuskyDriver()
    
    # We need to travel ~2.0 meters to get the arm close to the cube (at 3.0m)
    try:
        time.sleep(1) # Wait for connections
        node.move_forward(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()