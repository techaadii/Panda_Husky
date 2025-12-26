#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time

class HuskyBlindPick(Node):
    def __init__(self):
        super().__init__('husky_blind_pick')
        self.state = "DRIVE"
        self.start_pos = None
        self.dist_moved = 0.0

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/husky_velocity_controller/cmd_vel_unstamped', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/panda_hand_controller/joint_trajectory', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/husky_velocity_controller/odom', self.odom_callback, 10)

        self.get_logger().info("PROTOCOL: BLIND PICK INITIATED.")
        self.get_logger().info("Step 1: Driving to Target Zone...")

    def odom_callback(self, msg):
        if self.state != "DRIVE": return
        
        pos = msg.pose.pose.position
        if self.start_pos is None: self.start_pos = pos; return

        # Calculate distance driven
        self.dist_moved = math.sqrt((pos.x - self.start_pos.x)**2 + (pos.y - self.start_pos.y)**2)
        
        cmd = Twist()
        
        # TARGET: Cube is at x=3.0.
        # Robot Arm Base is offset by ~0.35m.
        # Ideal stopping point: ~0.65m away from cube.
        # Drive Distance = 3.0 - 0.65 - 0.35 = 2.0 meters roughly.
        target_dist = 2.15 

        if (target_dist - self.dist_moved) > 0.05:
            cmd.linear.x = 0.5 # Full speed ahead
            # Minor steering correction if needed (assuming straight line)
            cmd.angular.z = 0.0
        else:
            # STOP
            cmd.linear.x = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.state = "PICKING"
            self.get_logger().info("ARRIVED. Executing Blind Grasp Sequence...")
            self.perform_blind_pick()
            
        self.cmd_vel_pub.publish(cmd)

    def perform_blind_pick(self):
        # 1. Open Gripper
        self.move_gripper(0.04) 
        time.sleep(1)

        # 2. Move Arm to "Ready" (Hover over cube)
        # These joint angles place the end-effector roughly at x=0.6, z=0.2 (local)
        self.get_logger().info("Positioning Arm...")
        self.move_arm([0.0, 0.4, 0.0, -1.5, 0.0, 1.9, 0.785])
        time.sleep(4)

        # 3. Descend (Reach down)
        self.get_logger().info("Descend...")
        self.move_arm([0.0, 0.9, 0.0, -1.5, 0.0, 2.4, 0.785])
        time.sleep(4)

        # 4. Close Gripper (GRAB)
        self.get_logger().info("GRASPING...")
        self.move_gripper(0.0)
        time.sleep(2)

        # 5. Lift Up
        self.get_logger().info("LIFTING...")
        self.move_arm([0.0, -0.5, 0.0, -2.0, 0.0, 2.0, 0.785])
        time.sleep(2)
        
        self.get_logger().info("MISSION COMPLETE. I have the cube.")
        raise SystemExit

    def move_arm(self, pos):
        traj = JointTrajectory()
        traj.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        pt = JointTrajectoryPoint()
        pt.positions = pos
        pt.time_from_start.sec = 3
        traj.points.append(pt)
        self.arm_pub.publish(traj)

    def move_gripper(self, width):
        traj = JointTrajectory()
        traj.joint_names = ['panda_finger_joint1', 'panda_finger_joint2']
        pt = JointTrajectoryPoint()
        pt.positions = [width, width]
        pt.time_from_start.sec = 1
        traj.points.append(pt)
        self.gripper_pub.publish(traj)

def main():
    rclpy.init()
    try: rclpy.spin(HuskyBlindPick())
    except SystemExit: pass
    except KeyboardInterrupt: pass

if __name__ == '__main__': main()