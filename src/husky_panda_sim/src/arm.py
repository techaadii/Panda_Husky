#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import torch

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor


class HuskyThenPick(Node):

    def __init__(self):
        super().__init__('husky_then_pick')

        # ================= CONFIG =================
        self.GRASP_OFFSET = 0.12
        self.CAMERA_OFFSET = 0.2
        self.MAX_LIN_VEL = 0.15
        self.KP_DIST = 0.4
        self.KP_ANG = 0.002
        self.fx = 454.68

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # ================= SAM2 =================
        sam_model = build_sam2(
            "sam2_hiera_l.yaml",
            "/home/moonlab/ros_ws/src/husky_panda_sim/src/sam2_hiera_large.pt",
            device=self.device
        )
        self.predictor = SAM2ImagePredictor(sam_model)

        # ================= ROS =================
        self.bridge = CvBridge()

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            "/panda_arm_controller/joint_trajectory",
            10
        )

        self.img_color = None
        self.img_depth = None

        self.reached = False
        self.pick_sent = False

        color_sub = Subscriber(self, Image, "/husky_camera/image")
        depth_sub = Subscriber(self, Image, "/husky_camera/depth_image")

        ats = ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.05)
        ats.registerCallback(self.synced_cb)

        self.get_logger().info("Node started")

    # ================= IMAGE CALLBACK =================
    def synced_cb(self, color_msg, depth_msg):
        self.img_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        self.img_depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")

        if not self.reached:
            self.husky_control()
        elif not self.pick_sent:
            self.send_pick()

    # ================= BLUE SEED =================
    def get_blue_seed(self):
        hsv = cv2.cvtColor(self.img_color, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (100, 150, 50), (140, 255, 255))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < 150:
            return None
        M = cv2.moments(c)
        return np.array([[int(M["m10"]/M["m00"]),
                          int(M["m01"]/M["m00"])]], dtype=np.float32)

    # ================= HUSKY MOTION (UNCHANGED LOGIC) =================
    def husky_control(self):
        seed = self.get_blue_seed()
        if seed is None:
            self.cmd_pub.publish(Twist())
            return

        img_rgb = cv2.cvtColor(self.img_color, cv2.COLOR_BGR2RGB)

        with torch.inference_mode():
            self.predictor.set_image(img_rgb)
            masks, _, _ = self.predictor.predict(
                point_coords=seed,
                point_labels=np.array([1]),
                multimask_output=False
            )

        mask = masks[0]
        ys, xs = np.where(mask > 0)
        zs = self.img_depth[ys, xs]

        valid = np.isfinite(zs) & (zs > 0.05)
        if np.count_nonzero(valid) < 50:
            return

        dist = np.median(zs[valid]) - self.CAMERA_OFFSET
        self.get_logger().info(f"Distance: {dist:.3f}")

        if dist <= self.GRASP_OFFSET+0.005:
            self.cmd_pub.publish(Twist())
            self.reached = True
            self.get_logger().info("Reached grasp offset")
            return

        cmd = Twist()
        cmd.linear.x = min(self.KP_DIST * dist, self.MAX_LIN_VEL)
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    # ================= PICK =================
    def send_pick(self):
        self.get_logger().info("Sending pick trajectory")

        traj = JointTrajectory()
        traj.joint_names = [
            "panda_joint1", "panda_joint2", "panda_joint3",
            "panda_joint4", "panda_joint5", "panda_joint6",
            "panda_joint7"
        ]

        p = JointTrajectoryPoint()
        p.positions = [0.0, -0.6, 0.0, -2.2, 0.0, 2.0, 0.8]
        p.time_from_start.sec = 3

        traj.points.append(p)
        self.arm_pub.publish(traj)

        self.pick_sent = True
        self.get_logger().info("Pick command published")

# ================= MAIN =================
def main():
    rclpy.init()
    node = HuskyThenPick()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
