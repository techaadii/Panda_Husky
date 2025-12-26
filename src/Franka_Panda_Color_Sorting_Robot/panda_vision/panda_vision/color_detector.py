#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf_transformations
import math

# --- CORRECTED TOPICS (Based on your system checks) ---
# Base
ODOM_TOPIC = '/husky_velocity_controller/odom'
CMD_VEL_TOPIC = '/husky_velocity_controller/cmd_vel_unstamped'
# Arm
ARM_TOPIC = '/panda_arm_controller/joint_trajectory'
# Camera
CAMERA_TOPIC = '/camera/image_raw' # Ensure this matches your wrist camera topic

class HuskyPandaMission(Node):
    def __init__(self):
        super().__init__('husky_panda_mission')

        # --- STATE MACHINE ---
        self.state = "DRIVE" 
        self.target_distance = 2.1
        self.start_pos = None
        self.dist_moved = 0.0

        # --- NAVIGATION SETUP ---
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, 10)

        # --- ARM SETUP ---
        self.arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)

        # --- VISION SETUP ---
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, CAMERA_TOPIC, self.image_callback, 10)
        
        # TF2 Buffer for Coordinate Transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Camera Intrinsics (From your code)
        self.fx, self.fy = 585.0, 588.0
        self.cx, self.cy = 320.0, 160.0

        self.get_logger().info("MISSION START: Driving 2.1m...")

    def odom_callback(self, msg):
        """ Phase 1: Drive Logic """
        if self.state != "DRIVE": return

        pos = msg.pose.pose.position
        if self.start_pos is None:
            self.start_pos = pos
            return

        # Calculate Distance
        self.dist_moved = math.sqrt((pos.x - self.start_pos.x)**2 + (pos.y - self.start_pos.y)**2)

        cmd = Twist()
        if self.dist_moved < self.target_distance:
            cmd.linear.x = 0.5 # Drive forward
        else:
            # STOP and Switch to Arm
            cmd.linear.x = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info(f"Target Reached ({self.dist_moved:.2f}m). Stopping.")
            
            self.state = "ARM_READY"
            # TRIGGER ARM MOVEMENT IMMEDIATELY
            self.move_arm_to_ready() 
            
        self.cmd_vel_pub.publish(cmd)

    def image_callback(self, msg):
        """ Phase 3: Find Blue Cube & Align Arm """
        if self.state != "SCANNING": return # Wait until arm is in Ready pose

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except: return

        # HSV Blue Detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([90, 200, 200]), np.array([128, 255, 255]))
        
        # Clean noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 100:
                x, y, w, h = cv2.boundingRect(c)
                cx_pix = x + w//2
                cy_pix = y + h//2

                # --- 3D CALCULATION (Your Math) ---
                Z = 0.5 # Estimated distance
                X = (cy_pix - self.cy) * Z / self.fy
                Y = (cx_pix - self.cx) * Z / self.fx * -1
                
                try:
                    # Get Transform: Camera -> Panda Base
                    t = self.tf_buffer.lookup_transform("panda_link0", "camera_link", rclpy.time.Time())
                    
                    trans = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
                    rot = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
                    T = tf_transformations.quaternion_matrix(rot)
                    T[:3, 3] = trans
                    
                    pt_cam = np.array([X, Y, Z, 1.0])
                    pt_base = T @ pt_cam
                    
                    # Blue correction
                    pt_base[1] -= 0.0215 

                    self.get_logger().info(f"BLUE CUBE FOUND at [X:{pt_base[0]:.2f}, Y:{pt_base[1]:.2f}] relative to Arm Base.")
                    
                    # Phase 4: Align Arm
                    self.state = "FINISHED"
                    self.align_arm_to_cube(pt_base[0], pt_base[1])

                except Exception as e:
                    pass # TF not ready yet

        # Optional: Show what robot sees
        cv2.imshow("Robot Eye", frame)
        cv2.waitKey(1)

    def move_arm_to_ready(self):
        """ Move arm to a 'looking down' pose """
        self.get_logger().info("Moving Arm to Ready Position...")
        
        traj = JointTrajectory()
        traj.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        point = JointTrajectoryPoint()
        
        # Pose: Slightly bent, looking down
        point.positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
        point.time_from_start.sec = 2
        
        traj.points.append(point)
        self.arm_pub.publish(traj)
        
        # Allow time for move, then enable vision scanning
        self.create_timer(3.0, lambda: setattr(self, 'state', 'SCANNING'))

    def align_arm_to_cube(self, target_x, target_y):
        """ Rotates the base of the arm to point at the cube """
        self.get_logger().info("Aligning Arm to Cube...")
        
        # Calculate angle to target
        target_yaw = math.atan2(target_y, target_x)
        
        traj = JointTrajectory()
        traj.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        point = JointTrajectoryPoint()
        
        # Same pose, but rotated base (Joint 1)
        point.positions = [target_yaw, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
        point.time_from_start.sec = 2
        
        traj.points.append(point)
        self.arm_pub.publish(traj)
        self.get_logger().info(f"Command Sent! Rotating Base to {target_yaw:.2f} rad")

def main(args=None):
    rclpy.init(args=args)
    node = HuskyPandaMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()