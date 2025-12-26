#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint

# ========================================================================================
#  TUNING PARAMETERS
# ========================================================================================
DRIVE_DISTANCE = 2.09  # Your tested distance
GRIP_WIDTH     = 0.04  # Open width
GRIP_FORCE     = 500.0 # Max force
# ========================================================================================

class MobilePickPlaceMultiDip(Node):
    def __init__(self):
        super().__init__('mobile_pick_place_multidip')

        self.cmd_vel_pub = self.create_publisher(Twist, '/husky_velocity_controller/cmd_vel_unstamped', 10)
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/panda_arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd')
        
        self.get_logger().info("Waiting for Controllers...")
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info(">>> SYSTEM READY.")

    def drive_base(self, distance, speed=0.5):
        self.get_logger().info(f">>> BASE: Driving {distance}m...")
        msg = Twist()
        msg.linear.x = speed
        duration = distance / speed
        start = time.time()
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.05)
        msg.linear.x = 0.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(">>> BASE: Arrived.")
        time.sleep(1.0) 

    def move_gripper(self, width, effort=100.0):
        self.get_logger().info(f">>> GRIPPER: Width {width}...")
        goal = GripperCommand.Goal()
        goal.command.position = width
        goal.command.max_effort = effort
        self.gripper_client.send_goal_async(goal)
        time.sleep(2.0)

    def move_arm(self, joint_positions, duration=4.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.time_from_start.sec = int(duration)
        
        goal.trajectory.points = [point]
        
        self.get_logger().info(">>> ARM: Moving...")
        send_goal_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Arm Goal Rejected!")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(">>> ARM: Finished.")

    def run_mission(self):
        # 1. Drive
        self.drive_base(DRIVE_DISTANCE)

        # 2. Home Pose
        self.get_logger().info(">>> SEQUENCE: 1. Home")
        home_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
        self.move_arm(home_joints)

        # 3. Open Gripper
        self.move_gripper(GRIP_WIDTH)

        # 4. Pre-Grasp (Hover)
        self.get_logger().info(">>> SEQUENCE: 2. Pre-Grasp (Hover)")
        pre_grasp_joints = [0.0, 1.0, 0.0, -1.8, 0.0, 2.5, 0.785]
        self.move_arm(pre_grasp_joints, duration=4.0)

        # 5. Grasp Sequence (The Multi-Dip)
        
        # Level 1: Current Height (Likely Hovering)
        self.get_logger().info(">>> SEQUENCE: 3a. Grasp Level 1 (High)")
        grasp_joints_1 = [0.0, 1.5, 0.0, -1.9, 0.0, 3.0, 0.785]
        self.move_arm(grasp_joints_1, duration=3.0)

        # Level 2: Lower (Leaning Shoulder Forward)
        self.get_logger().info(">>> SEQUENCE: 3b. Grasp Level 2 (Lower)")
        grasp_joints_2 = [0.0, 1.58, 0.0, -1.85, 0.0, 3.1, 0.785]
        self.move_arm(grasp_joints_2, duration=2.0)

        # Level 3: Lowest (Scraping the floor)
        self.get_logger().info(">>> SEQUENCE: 3c. Grasp Level 3 (Floor)")
        grasp_joints_3 = [0.0, 1.65, 0.0, -1.75, 0.0, 3.2, 0.785]
        self.move_arm(grasp_joints_3, duration=2.0)

        # 6. Close Gripper (Full Force)
        self.get_logger().info(">>> GRIPPER: CLOSING")
        self.move_gripper(0.0, effort=GRIP_FORCE)
        time.sleep(1.0) 

        # 7. Lift (Straight Up first to avoid dragging)
        self.get_logger().info(">>> SEQUENCE: 4. Lifting")
        self.move_arm(home_joints, duration=5.0)

        self.get_logger().info(">>> MISSION COMPLETE")

def main(args=None):
    rclpy.init(args=args)
    node = MobilePickPlaceMultiDip()
    try:
        node.run_mission()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()