#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class HuskyNavAndPick(Node):

    def __init__(self):
        super().__init__("husky_nav_pick")

        # ---------------- NAVIGATION ONLY ----------------
        self.target_x = 2.5
        self.current_x = None

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_cb,
            10
        )

    # ---------------- ODOM CALLBACK ----------------
    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x

    # ---------------- NAVIGATION PHASE ----------------
    def navigate_to_offset(self):
        self.get_logger().info("Waiting for odom...")

        while self.current_x is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Starting navigation")

        while rclpy.ok():
            error = self.target_x - self.current_x

            if abs(error) < 0.05:
                break

            cmd = Twist()
            cmd.linear.x = max(min(0.4 * error, 0.4), 0.05)
            self.cmd_pub.publish(cmd)

            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        # FULL STOP
        self.cmd_pub.publish(Twist())
        self.get_logger().info("Navigation complete — Husky stopped")

    # ---------------- MANIPULATION PHASE ----------------
    def init_manipulation(self):
        self.get_logger().info("Initializing manipulation...")

        # IMPORT MOVEIT **ONLY NOW**
        from pymoveit2.moveit2 import MoveIt2
        from pymoveit2.gripper_interface import GripperInterface
        from pymoveit2.robots import panda

        self.moveit = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM
        )

        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS
        )

        self.get_logger().info("MoveIt initialized")

    def pick_cube(self):
        self.get_logger().info("Picking cube...")

        # Example pose (replace with depth-based pose)
        self.moveit.move_to_pose(
            position=[0.6, 0.0, 0.3],
            quat_xyzw=[0.0, 1.0, 0.0, 0.0]
        )
        self.moveit.wait_until_executed()

        self.gripper.close()
        time.sleep(1)

        self.moveit.move_to_pose(
            position=[0.6, 0.0, 0.5],
            quat_xyzw=[0.0, 1.0, 0.0, 0.0]
        )
        self.moveit.wait_until_executed()

        self.get_logger().info("Cube picked successfully")


# ---------------- MAIN ----------------
def main():
    rclpy.init()

    node = HuskyNavAndPick()

    # PHASE 1
    node.navigate_to_offset()

    # PHASE 2
    node.init_manipulation()
    node.pick_cube()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
