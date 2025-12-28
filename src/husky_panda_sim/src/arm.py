#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import torch
from threading import Thread
import time

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

from pymoveit2 import MoveIt2, MoveIt2State, GripperInterface
from pymoveit2.robots import panda as robot
from rclpy.callback_groups import ReentrantCallbackGroup


class HuskyPickPlace(Node):
    def __init__(self):
        super().__init__('husky_pick_place_node')

        # ===================== CONFIG =====================
        self.GRASP_OFFSET = 0.01       # stopping distance
        self.CAMERA_OFFSET = 0.35
        self.MAX_LIN_VEL = 0.15
        self.KP_DIST = 0.4
        self.KP_ANG = 0.002

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # ===================== SAM2 =====================
        checkpoint = "/home/moonlab/ros_ws/src/husky_panda_sim/src/sam2_hiera_large.pt"
        model_cfg = "sam2_hiera_l.yaml"
        sam2_model = build_sam2(model_cfg, checkpoint, device=self.device)
        self.predictor = SAM2ImagePredictor(sam2_model)

        # ===================== ROS =====================
        self.bridge = CvBridge()
        self.img_color = None
        self.img_depth = None
        self.data_ready = False
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        color_sub = Subscriber(self, Image, '/husky_camera/image')
        depth_sub = Subscriber(self, Image, '/husky_camera/depth_image')
        ats = ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=10, slop=0.05)
        ats.registerCallback(self.synced_cb)

        # OpenCV windows
        cv2.namedWindow("RGB", cv2.WINDOW_NORMAL)
        cv2.namedWindow("SAM Mask", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Depth Mask", cv2.WINDOW_NORMAL)

        # ===================== PHASE CONTROL =====================
        self.phase_navigation_done = False
        self.phase_manipulation_done = False

        # Start navigation thread
        nav_thread = Thread(target=self.navigation_loop)
        nav_thread.start()

    # ===================== IMAGE CALLBACK =====================
    def synced_cb(self, color_msg, depth_msg):
        self.img_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        self.img_depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        self.data_ready = True

    # ===================== BLUE SEED =====================
    def get_blue_seed(self):
        hsv = cv2.cvtColor(self.img_color, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([100, 150, 50]), np.array([140, 255, 255]))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts: return None
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < 150: return None
        M = cv2.moments(c)
        return np.array([[int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])]], dtype=np.float32)

    # ===================== DISTANCE ESTIMATION =====================
    def estimate_distance_and_error(self):
        seed = self.get_blue_seed()
        if seed is None: return None

        img_rgb = cv2.cvtColor(self.img_color, cv2.COLOR_BGR2RGB)
        with torch.inference_mode(), torch.autocast("cuda"):
            self.predictor.set_image(img_rgb)
            masks, _, _ = self.predictor.predict(point_coords=seed, point_labels=np.array([1]), multimask_output=False)

        sam_mask = masks[0].astype(np.uint8)
        self.visualize(self.img_color, self.img_depth, sam_mask, seed)

        ys, xs = np.where(sam_mask > 0)
        zs = self.img_depth[ys, xs]
        valid = np.isfinite(zs) & (zs > 0.05)
        if np.count_nonzero(valid) < 50: return None

        xs, zs = xs[valid], zs[valid]
        dist = np.median(zs)
        cx, fx = self.img_color.shape[1]/2.0, 454.68
        lateral_error = np.mean((xs - cx)*zs/fx)
        return dist, lateral_error

    # ===================== VISUALIZATION =====================
    def visualize(self, rgb, depth, sam_mask, seed):
        rgb_vis = rgb.copy()
        cv2.circle(rgb_vis, tuple(seed[0].astype(int)), 5, (0,0,255), -1)
        sam_color = np.zeros_like(rgb)
        sam_color[:,:,1] = sam_mask*255
        sam_overlay = cv2.addWeighted(rgb, 0.7, sam_color, 0.3, 0)
        depth_masked = np.zeros_like(depth)
        depth_masked[sam_mask>0] = depth[sam_mask>0]
        depth_vis = cv2.normalize(depth_masked,None,0,255,cv2.NORM_MINMAX).astype(np.uint8)
        cv2.imshow("RGB", rgb_vis)
        cv2.imshow("SAM Mask", sam_overlay)
        cv2.imshow("Depth Mask", depth_vis)
        cv2.waitKey(1)

    # ===================== NAVIGATION LOOP =====================
    def navigation_loop(self):
        rate = self.create_rate(10)
        self.get_logger().info("Starting navigation phase...")
        while rclpy.ok() and not self.phase_navigation_done:
            if self.data_ready:
                self.data_ready = False
                result = self.estimate_distance_and_error()
                if result is None:
                    self.cmd_pub.publish(Twist())
                    rate.sleep()
                    continue

                raw_dist, err_y = result
                dist = raw_dist - 0.35
                self.get_logger().info(f"Husky distance: {dist:.3f} m")

                if dist <= self.GRASP_OFFSET:
                    self.cmd_pub.publish(Twist())  # stop
                    self.get_logger().info("Husky reached target distance. Stopping navigation.")
                    self.phase_navigation_done = True
                    break

                cmd = Twist()
                cmd.linear.x = min(self.KP_DIST*(dist-self.GRASP_OFFSET), self.MAX_LIN_VEL)
                cmd.angular.z = -self.KP_ANG*err_y
                self.cmd_pub.publish(cmd)
            rate.sleep()

        # Once navigation is done, start manipulation phase
        self.manipulation_phase()

    # ===================== MANIPULATION PHASE =====================
    def manipulation_phase(self):
        self.get_logger().info("Starting manipulation phase...")

        # ===================== MOVEIT2 =====================
        callback_group = ReentrantCallbackGroup()
        moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=callback_group
        )
        gripper = GripperInterface(
            node=self,
            gripper_joint_names=robot.gripper_joint_names(),
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            callback_group=callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd"
        )

        # Wait for joint states
        rate = self.create_rate(10)
        while rclpy.ok():
            try:
                joints = moveit2.get_joint_positions()
                if joints is not None:
                    self.get_logger().info("Panda joint states available.")
                    break
            except Exception:
                pass
            rate.sleep()

        # ===================== PICK & PLACE =====================
        self.get_logger().info("PICK SEQUENCE STARTING")
        pick_position = [0.5, 0.0, 0.25]
        pick_quat = [1.0, 0.0, 0.0, 0.0]
        moveit2.move_to_pose(position=pick_position, quat_xyzw=pick_quat, cartesian=False)
        moveit2.wait_until_executed()
        gripper.close()
        gripper.wait_until_executed()

        self.get_logger().info("PLACE SEQUENCE STARTING")
        place_position = [0.25, 0.0, 0.25]
        place_quat = [1.0, 0.0, 0.0, 0.0]
        moveit2.move_to_pose(position=place_position, quat_xyzw=place_quat, cartesian=False)
        moveit2.wait_until_executed()
        gripper.open()
        gripper.wait_until_executed()

        self.get_logger().info("MANIPULATION COMPLETED")
        self.phase_manipulation_done = True


def main():
    rclpy.init()
    node = HuskyPickPlace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
