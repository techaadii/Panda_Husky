#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time

# NumPy Patch
if not hasattr(np, 'float'): np.float = float
import tf2_ros
import tf_transformations

# --- CONFIGURATION ---
CAMERA_TOPIC = '/wrist_camera/image' 

class HuskyVisualDebug(Node):
    def __init__(self):
        super().__init__('husky_visual_debug')
        self.state = "DRIVE"
        self.start_pos = None
        self.dist_moved = 0.0

        # QoS for Bridge
        self.qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/husky_velocity_controller/cmd_vel_unstamped', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        self.odom_sub = self.create_subscription(Odometry, '/husky_velocity_controller/odom', self.odom_callback, 10)
        
        # Subscribe
        self.image_sub = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, qos_profile=self.qos_policy)

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Intrinsics
        self.fx, self.fy = 585.0, 588.0
        self.cx, self.cy = 320.0, 160.0
        
        self.get_logger().info("DEBUGGER START. If no window appears, check the Bridge!")

    def odom_callback(self, msg):
        if self.state != "DRIVE": return
        pos = msg.pose.pose.position
        if self.start_pos is None: self.start_pos = pos; return

        self.dist_moved = math.sqrt((pos.x - self.start_pos.x)**2 + (pos.y - self.start_pos.y)**2)
        cmd = Twist()

        # Drive 2.25m
        if (2.25 - self.dist_moved) > 0.05:
            cmd.linear.x = 0.4
        else:
            cmd.linear.x = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info("ARRIVED. Moving Arm to NEW Search Pose...")
            self.state = "SCANNING"
            
            # --- NEW POSE FOR EYE-IN-HAND ---
            # This pose extends the arm out and points the palm DOWN/FORWARD
            # Joint 2 (-1.0): Shoulder lean
            # Joint 4 (-2.0): Elbow bend
            # Joint 6 (1.57): Wrist neutral (pointing with forearm)
            self.move_arm([0.0, -0.7, 0.0, -2.2, 0.0, 1.6, 0.785])
            
        self.cmd_vel_pub.publish(cmd)

    def image_callback(self, msg):
        # ALWAYS PROCESS IMAGE TO DEBUG
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # --- VISUALIZATION ---
            # This window MUST appear. If not, the callback isn't triggering.
            cv2.imshow("What Robot Sees", frame)
            cv2.waitKey(1)

            if self.state != "SCANNING": return

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # Wide Blue Range
            mask = cv2.inRange(hsv, np.array([80, 50, 20]), np.array([140, 255, 255]))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                print(f"Blue Area: {area:.0f}", end='\r')

                if area > 200:
                    self.get_logger().info("\nTARGET LOCKED. Coordinates calculated.")
                    self.state = "PICKING"
                    self.process_pick(c)
        except Exception as e:
            self.get_logger().error(f"IMAGE ERROR: {e}")

    def process_pick(self, cnt):
        x, y, w, h = cv2.boundingRect(cnt)
        cx_pix, cy_pix = x + w//2, y + h//2
        
        # Adjusted height for Eye-in-Hand
        Z = 0.45 
        
        Y_real = (cx_pix - self.cx) * Z / self.fx * -1 
        X_real = (cy_pix - self.cy) * Z / self.fy
        
        try:
            # Ensure this matches your URDF link name
            t = self.tf_buffer.lookup_transform("panda_link0", "wrist_camera_link", rclpy.time.Time())
            trans = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
            rot = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            T = tf_transformations.quaternion_matrix(rot)
            T[:3, 3] = trans
            pt = T @ np.array([X_real, Y_real, Z, 1.0])
            
            yaw = math.atan2(pt[1], pt[0]) 
            self.perform_sequence(yaw)
        except Exception as e:
            self.get_logger().warn(f"TF Error: {e}")

    def perform_sequence(self, yaw):
        self.get_logger().info("EXECUTING PICK...")
        
        # 1. Align
        self.move_arm([yaw, -0.7, 0.0, -2.2, 0.0, 1.6, 0.785])
        time.sleep(3)
        
        # 2. Descend 
        # Note: Eye-in-hand needs to be careful not to smash camera
        self.move_arm([yaw, 0.8, 0.0, -1.5, 0.0, 2.5, 0.785])
        time.sleep(4)
        
        # 3. Lift
        self.move_arm([yaw, -1.0, 0.0, -2.5, 0.0, 3.0, 0.785])
        
        self.get_logger().info("MISSION SUCCESS.")
        raise SystemExit

    def move_arm(self, pos):
        traj = JointTrajectory()
        traj.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        pt = JointTrajectoryPoint()
        pt.positions = pos
        pt.time_from_start.sec = 2
        traj.points.append(pt)
        self.arm_pub.publish(traj)

def main():
    rclpy.init()
    try: rclpy.spin(HuskyVisualDebug())
    except SystemExit: pass
    except KeyboardInterrupt: pass

if __name__ == '__main__': main()