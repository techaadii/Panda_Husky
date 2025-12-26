#!/usr/bin/env python3
"""
Flicker-resistant visual servoing system.
Uses frame buffering and extreme tolerance to handle camera flickering.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from collections import deque

class VisualGraspNode(Node):
    def __init__(self):
        super().__init__('visual_grasp_node')
        
        # === PARAMETERS ===
        self.CLOSE_ENOUGH_AREA = 3000
        self.MIN_DETECTION_AREA = 50       # Very small - cube appears small
        self.SEARCH_ROTATION_SPEED = 0.12
        self.MAX_LINEAR_SPEED = 0.20
        self.TURN_GAIN = 0.0015
        
        # === EXTREME TOLERANCE FOR FLICKERING ===
        self.FRAMES_TO_CONFIRM = 1         # Lock immediately!
        self.FRAMES_TO_LOSE = 30           # Hold lock for 30 missing frames!
        
        # Frame buffering to handle flicker
        self.frame_buffer = deque(maxlen=3)
        self.detection_buffer = deque(maxlen=5)
        
        # === ROS SETUP ===
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/husky_camera/image', self.camera_callback, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/panda_hand_controller/joint_trajectory', 10)
        
        # === STATE ===
        self.state = "SEARCHING"
        self.cube_locked = False
        self.cube_center_x = 320
        self.cube_area = 0
        self.image_width = 640
        self.image_center_x = 320
        
        # Persistence
        self.frames_with_detection = 0
        self.frames_without_detection = 0
        
        # VERY heavy smoothing for flicker
        self.smoothed_x = 320
        self.smoothed_area = 0
        
        self.frame_count = 0
        
        self.get_logger().info("=== Flicker-Resistant Visual Grasp Started ===")
        self.get_logger().info(f"EXTREME tolerance: Lock={self.FRAMES_TO_CONFIRM}, Hold={self.FRAMES_TO_LOSE}")
        
    def camera_callback(self, msg):
        """Process camera images with flicker handling"""
        if self.state == "DONE":
            return
        
        self.frame_count += 1
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Check if frame is valid (not corrupted/flickering)
            if frame is None or frame.size == 0:
                self.get_logger().warn("Corrupted frame, skipping...")
                return
                
            h, w, _ = frame.shape
            if w < 100 or h < 100:
                self.get_logger().warn("Invalid frame size, skipping...")
                return
            
            self.image_width = w
            self.image_center_x = w // 2
            
            # Buffer frames
            self.frame_buffer.append(frame.copy())
            
            # Only process every 2nd frame to reduce load
            if self.frame_count % 2 != 0:
                return
            
            # Detect cube with multiple attempts
            detected, center_x, area = self.detect_cube_robust(frame)
            
            # Buffer detection results
            self.detection_buffer.append(detected)
            
            # Use voting: if majority of recent frames detected cube, count it as detected
            detection_votes = sum(self.detection_buffer)
            majority_detected = detection_votes >= 2  # At least 2 out of 5 recent frames
            
            # === PERSISTENCE WITH VOTING ===
            if majority_detected:
                self.frames_with_detection += 1
                self.frames_without_detection = 0
                
                # Update smoothed values (EXTREME smoothing for flicker)
                if detected:  # Only update if THIS frame had detection
                    alpha = 0.90  # Extreme smoothing
                    self.smoothed_x = int(alpha * self.smoothed_x + (1 - alpha) * center_x)
                    self.smoothed_area = int(alpha * self.smoothed_area + (1 - alpha) * area)
                
                # Instant lock
                if self.frames_with_detection >= self.FRAMES_TO_CONFIRM:
                    if not self.cube_locked:
                        self.get_logger().info(f"✓✓✓ CUBE LOCKED! X={self.smoothed_x}, Area={self.smoothed_area}")
                    self.cube_locked = True
                    self.cube_center_x = self.smoothed_x
                    self.cube_area = self.smoothed_area
            else:
                self.frames_without_detection += 1
                
                # Keep lock with memory
                if self.cube_locked and self.frames_without_detection < self.FRAMES_TO_LOSE:
                    # Use memory
                    self.cube_center_x = self.smoothed_x
                    self.cube_area = self.smoothed_area
                elif self.frames_without_detection >= self.FRAMES_TO_LOSE:
                    if self.cube_locked:
                        self.get_logger().warn(f"✗✗✗ LOST CUBE after {self.frames_without_detection} frames")
                    self.cube_locked = False
                    self.frames_with_detection = 0
            
            # === VISUALIZATION ===
            vis_frame = frame.copy()
            
            if self.cube_locked:
                # Draw target
                cv2.circle(vis_frame, (self.cube_center_x, h//2), 20, (0, 255, 0), 3)
                cv2.line(vis_frame, (self.cube_center_x, 0), (self.cube_center_x, h), (0, 255, 0), 2)
                
                status = f"LOCKED | X:{self.cube_center_x} | Area:{self.cube_area}"
                color = (0, 255, 0)
                
                if not detected:
                    status += f" | MEM({self.frames_without_detection})"
                    color = (0, 255, 255)
                    
                cv2.putText(vis_frame, status, (10, 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            else:
                cv2.putText(vis_frame, f"SEARCHING... Votes:{detection_votes}/5", (10, 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            cv2.putText(vis_frame, f"State: {self.state} | Frame: {self.frame_count}", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.imshow("Robot Vision", vis_frame)
            cv2.waitKey(1)
            
            # Control
            self.control_robot()
            
        except Exception as e:
            self.get_logger().error(f"Frame processing error: {e}")
    
    def detect_cube_robust(self, frame):
        """
        Robust detection targeting the small dark blue square.
        Returns: (detected: bool, center_x: int, area: float)
        """
        try:
            h, w, _ = frame.shape
            
            # Create ROI - focus on center/lower area
            roi = frame.copy()
            roi[0:int(h*0.2), :] = 0  # Remove top 20%
            
            # Multiple color space approaches
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            # Method 1: Dark blue (what we see in the image)
            # The cube appears as dark blue/navy
            lower_dark_blue = np.array([100, 50, 20])   # Very dark blue
            upper_dark_blue = np.array([130, 255, 150])
            mask1 = cv2.inRange(hsv, lower_dark_blue, upper_dark_blue)
            
            # Method 2: Any blue
            lower_blue = np.array([90, 30, 20])
            upper_blue = np.array([140, 255, 255])
            mask2 = cv2.inRange(hsv, lower_blue, upper_blue)
            
            # Method 3: Dark objects
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, mask3 = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
            
            # Combine masks
            mask = cv2.bitwise_or(mask1, mask2)
            mask = cv2.bitwise_or(mask, mask3)
            
            # Clean up
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Show masks
            cv2.imshow("Combined Mask", mask)
            cv2.imshow("Dark Blue Mask", mask1)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) == 0:
                return False, 0, 0
            
            # Filter contours by size and shape
            valid_contours = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.MIN_DETECTION_AREA:
                    continue
                
                # Check if roughly square-ish (cube should be somewhat square)
                x, y, bw, bh = cv2.boundingRect(cnt)
                aspect_ratio = float(bw) / bh if bh > 0 else 0
                
                # Accept if somewhat square (0.5 to 2.0 aspect ratio)
                if 0.3 < aspect_ratio < 3.0:
                    valid_contours.append(cnt)
            
            if len(valid_contours) == 0:
                return False, 0, 0
            
            # Get largest valid contour
            largest = max(valid_contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            
            # Calculate center
            M = cv2.moments(largest)
            if M['m00'] == 0:
                return False, 0, 0
                
            center_x = int(M['m10'] / M['m00'])
            
            return True, center_x, area
            
        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")
            return False, 0, 0
    
    def control_robot(self):
        """Control logic"""
        if self.state == "SEARCHING":
            self.search_behavior()
        elif self.state == "APPROACHING":
            self.approach_behavior()
    
    def search_behavior(self):
        """Rotate to find cube"""
        cmd = Twist()
        
        if not self.cube_locked:
            cmd.angular.z = self.SEARCH_ROTATION_SPEED
            self.vel_pub.publish(cmd)
            
            if self.frame_count % 30 == 0:
                self.get_logger().info("Searching...")
        else:
            self.get_logger().info("━━━ FOUND! Starting approach ━━━")
            self.state = "APPROACHING"
    
    def approach_behavior(self):
        """Drive toward cube"""
        cmd = Twist()
        
        if not self.cube_locked:
            self.get_logger().warn("━━━ Lost, back to search ━━━")
            self.state = "SEARCHING"
            return
        
        # Check distance
        if self.cube_area >= self.CLOSE_ENOUGH_AREA:
            self.get_logger().info(f"━━━ GRASPING! Area={self.cube_area} ━━━")
            self.stop_robot()
            self.state = "GRASPING"
            self.execute_grasp_sequence()
            return
        
        # Steering
        error_x = self.image_center_x - self.cube_center_x
        cmd.angular.z = self.TURN_GAIN * error_x
        
        # Speed
        if abs(error_x) < 60:
            cmd.linear.x = self.MAX_LINEAR_SPEED
            if self.frame_count % 30 == 0:
                self.get_logger().info(f"→ Forward | Area:{self.cube_area}")
        else:
            alignment = 1.0 - (abs(error_x) / self.image_width)
            cmd.linear.x = self.MAX_LINEAR_SPEED * alignment * 0.6
            cmd.linear.x = max(0.05, cmd.linear.x)
            if self.frame_count % 30 == 0:
                self.get_logger().info(f"↻ Steering | Err:{error_x}")
        
        self.vel_pub.publish(cmd)
    
    def stop_robot(self):
        cmd = Twist()
        self.vel_pub.publish(cmd)
        time.sleep(0.5)
    
    def execute_grasp_sequence(self):
        self.get_logger().info("╔═══════════════════════════╗")
        self.get_logger().info("║   GRASP SEQUENCE START    ║")
        self.get_logger().info("╚═══════════════════════════╝")
        
        self.command_gripper(0.04)
        time.sleep(1.5)
        
        self.command_arm([0.0, 1.2, 0.0, -2.0, 0.0, 2.4, 0.785], 4.0)
        time.sleep(5.0)
        
        self.command_gripper(0.0)
        time.sleep(2.0)
        
        self.command_arm([0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.785], 3.0)
        time.sleep(4.0)
        
        self.state = "DONE"
        self.get_logger().info("╔═══════════════════════════╗")
        self.get_logger().info("║    MISSION COMPLETE! 🎉   ║")
        self.get_logger().info("╚═══════════════════════════╝")
    
    def command_arm(self, positions, duration=3.0):
        msg = JointTrajectory()
        msg.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        msg.points.append(point)
        self.arm_pub.publish(msg)
    
    def command_gripper(self, width):
        msg = JointTrajectory()
        msg.joint_names = ['panda_finger_joint1', 'panda_finger_joint2']
        point = JointTrajectoryPoint()
        point.positions = [width, width]
        point.time_from_start.sec = 1
        msg.points.append(point)
        self.gripper_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisualGraspNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()