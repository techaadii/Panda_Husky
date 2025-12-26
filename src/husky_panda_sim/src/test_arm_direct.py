#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class TestArm(Node):
    def __init__(self):
        super().__init__('test_arm_direct')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/panda_arm_controller/follow_joint_trajectory')

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        
        point = JointTrajectoryPoint()
        # Move to a simple "Home" pose
        point.positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
        point.time_from_start.sec = 5  # Take 5 seconds to move
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = TestArm()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()