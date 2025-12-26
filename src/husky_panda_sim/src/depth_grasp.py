# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import cv2
# import numpy as np
# import torch
# import time

# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge

# import open3d as o3d
# from sam2.build_sam import build_sam2
# from sam2.sam2_image_predictor import SAM2ImagePredictor

# class DebugReachNode(Node):
#     def __init__(self):
#         super().__init__('debug_reach_node')

#         # --- 1. SETTINGS ---
#         self.GRASP_OFFSET = 0.65      # Target distance from camera
#         self.LIN_SPEED = 0.15         # Increased slightly to ensure motion
        
#         # --- 2. SAM2 SETUP ---
#         self.get_logger().info("Loading SAM2...")
#         checkpoint = "/home/moonlab/ros_ws/src/husky_panda_sim/src/sam2_hiera_large.pt"
#         model_cfg = "sam2_hiera_l.yaml"
#         self.predictor = SAM2ImagePredictor(build_sam2(model_cfg, checkpoint))

#         self.bridge = CvBridge()
#         self.img_color = None
#         self.img_depth = None
#         self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         self.create_subscription(Image, '/husky_camera/image', self.color_cb, 10)
#         self.create_subscription(Image, '/husky_camera/depth_image', self.depth_cb, 10)

#         self.target_dist = 999.0
#         self.target_found = False

#     def color_cb(self, msg): self.img_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#     def depth_cb(self, msg): self.img_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")

#     def run_perception(self):
#         if self.img_color is None or self.img_depth is None: return False

#         # Visualizing raw depth for you to see
#         # Normalize depth for display (0 to 255)
#         depth_display = cv2.normalize(self.img_depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
#         depth_colormap = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)

#         # SAM2 Pass
#         img_rgb = cv2.cvtColor(self.img_color, cv2.COLOR_BGR2RGB)
#         self.predictor.set_image(img_rgb)
        
#         # PROMPT: We search a small grid if center fails
#         input_point = np.array([[320, 380]]) 
#         input_label = np.array([1])
#         masks, scores, _ = self.predictor.predict(input_point, input_label, multimask_output=False)
#         mask = masks[0].astype(np.uint8) * 255

#         # Extract Depth
#         ys, xs = np.where(mask > 0)
#         if len(xs) < 10: 
#             self.show_windows(depth_colormap, mask)
#             return False

#         z_vals = self.img_depth[ys, xs]
#         valid = (z_vals > 0.01) & (z_vals < 5.0)
#         if not np.any(valid): 
#             self.show_windows(depth_colormap, mask)
#             return False

#         self.target_dist = np.mean(z_vals[valid])
#         self.target_found = True

#         # Draw centroid on RGB for debug
#         debug_rgb = self.img_color.copy()
#         cv2.circle(debug_rgb, (int(np.mean(xs)), int(np.mean(ys))), 10, (0, 255, 0), -1)
#         self.show_windows(depth_colormap, mask, debug_rgb)
        
#         return True

#     def show_windows(self, depth, mask, rgb=None):
#         """Displays the diagnostic images at each timestep."""
#         if rgb is not None:
#             cv2.imshow("1. RGB (Centroid)", rgb)
#         cv2.imshow("2. Depth (Heatmap)", depth)
#         cv2.imshow("3. SAM2 Mask", mask)
#         cv2.waitKey(1)

#     def run_control(self):
#         cmd = Twist()
#         # Perception phase (Robot stops here)
#         self.vel_pub.publish(Twist()) 
#         success = self.run_perception()

#         if not success:
#             if not self.target_found:
#                 cmd.angular.z = 0.2
#                 self.vel_pub.publish(cmd)
#             return

#         # Distance logic
#         if self.target_dist <= self.GRASP_OFFSET:
#             self.get_logger().info(f"STOPPED. Final Distance: {self.target_dist:.3f}m")
#             self.vel_pub.publish(Twist())
#         else:
#             self.get_logger().info(f"Target: {self.target_dist:.3f}m. Moving...")
#             cmd.linear.x = self.LIN_SPEED
#             self.vel_pub.publish(cmd)
#             # We move for a short burst to ensure the physics engine registers it
#             time.sleep(0.4) 
#             self.vel_pub.publish(Twist()) # Stop before next perception

# def main():
#     rclpy.init()
#     node = DebugReachNode()
#     while rclpy.ok():
#         rclpy.spin_once(node, timeout_sec=0.05)
#         node.run_control()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()





# ##!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import cv2
# import numpy as np
# import time

# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge
# from sam2.build_sam import build_sam2
# from sam2.sam2_image_predictor import SAM2ImagePredictor

# class FinalHuskyFixNode(Node):
#     def __init__(self):
#         super().__init__('final_husky_fix_node')

#         # --- 1. CRITICAL THRESHOLDS ---
#         self.GRASP_OFFSET = 0.65       # Target distance in meters
#         self.is_stopped = False
        
#         # --- 2. CAMERA CALIBRATION (Matched to your SDF) ---
#         # For 640x480 with ~1.5 rad FOV, fx is approx 454.68
#         self.fx = 454.686
#         self.cx, self.cy = 320, 240

#         # --- 3. SAM2 SETUP ---
#         self.get_logger().info("Loading SAM2...")
#         checkpoint = "/home/moonlab/ros_ws/src/husky_panda_sim/src/sam2_hiera_large.pt"
#         model_cfg = "sam2_hiera_l.yaml"
#         self.predictor = SAM2ImagePredictor(build_sam2(model_cfg, checkpoint))

#         self.bridge = CvBridge()
#         self.img_color = None
#         self.img_depth = None
#         self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         self.create_subscription(Image, '/husky_camera/image', self.color_cb, 10)
#         self.create_subscription(Image, '/husky_camera/depth_image', self.depth_cb, 10)

#         self.current_dist = 999.0

#     def color_cb(self, msg): self.img_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
#     def depth_cb(self, msg): 
#         # Force conversion to 32-bit float meters
#         self.img_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")

#     def run_perception(self):
#         if self.img_color is None or self.img_depth is None: return False

#         # 1. SAM2 Inference
#         img_rgb = cv2.cvtColor(self.img_color, cv2.COLOR_BGR2RGB)
#         self.predictor.set_image(img_rgb)
#         input_point = np.array([[320, 360]]) 
#         input_label = np.array([1])
#         masks, _, _ = self.predictor.predict(input_point, input_label, multimask_output=False)
#         mask = masks[0].astype(np.uint8)

#         # 2. RAW DEPTH CLEANING (The Fix)
#         # We create a 'Clean' depth map where everything past 5 meters is ignored
#         clean_depth = np.nan_to_num(self.img_depth, nan=5.0, posinf=5.0)
#         clean_depth = np.clip(clean_depth, 0.1, 5.0) 

#         # 3. MASKED EXTRACTION
#         ys, xs = np.where(mask > 0)
#         if len(xs) < 10: 
#             self.debug_display(clean_depth, mask)
#             return False

#         z_vals = clean_depth[ys, xs]
#         # Only consider values that are clearly not the 'far plane'
#         valid_z = z_vals[z_vals < 4.9] 

#         if len(valid_z) == 0:
#             self.target_dist = 999.0
#             self.debug_display(clean_depth, mask)
#             return False

#         # Use the Median for the most stable reading
#         self.target_dist = np.median(valid_z)
        
#         self.debug_display(clean_depth, mask, found=True)
#         return True

#     def debug_display(self, clean_depth, mask, found=False):
#         # Normalize the CLEANED depth so the cube is visible
#         depth_vis = cv2.normalize(clean_depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
#         depth_color = cv2.applyColorMap(255 - depth_vis, cv2.COLORMAP_JET) # Invert so close is Red
        
#         status_color = (0, 255, 0) if found else (0, 0, 255)
#         cv2.putText(depth_color, f"DIST: {self.target_dist:.2f}m", (20, 40), 
#                     cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)
        
#         cv2.imshow("VISUAL PROOF: Depth Map", depth_color)
#         cv2.imshow("VISUAL PROOF: SAM2 Mask", mask * 255)
#         cv2.waitKey(1)

#     def run_control(self):
#         if self.is_stopped: return

#         # Stop base to calculate
#         self.vel_pub.publish(Twist()) 
#         if not self.run_perception(): return

#         self.get_logger().info(f"Reported Distance: {self.current_dist:.3f}m")

#         # --- THE STOP CONDITION ---
#         if self.current_dist <= self.GRASP_OFFSET:
#             self.get_logger().info("!!! STOPPED AT OFFSET !!!")
#             self.vel_pub.publish(Twist())
#             self.is_stopped = True
#         else:
#             # Step forward
#             cmd = Twist()
#             cmd.linear.x = 0.15 
#             self.vel_pub.publish(cmd)
#             time.sleep(0.4) 
#             self.vel_pub.publish(Twist())

# def main():
#     rclpy.init()
#     node = FinalHuskyFixNode()
#     while rclpy.ok():
#         rclpy.spin_once(node, timeout_sec=0.05)
#         node.run_control()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# 

#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import torch
import time

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

class GpuSyncedReach(Node):
    def __init__(self):
        super().__init__('gpu_synced_reach_node')

        # --- 1. CONFIGURATION ---
        self.GRASP_OFFSET = 0.65       # Final target distance
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # --- 2. SAM2 GPU LOAD ---
        self.get_logger().info(f"Moving SAM2 to {self.device}...")
        checkpoint = "/home/moonlab/ros_ws/src/husky_panda_sim/src/sam2_hiera_large.pt"
        model_cfg = "sam2_hiera_l.yaml"
        # Load model directly to GPU
        sam2_model = build_sam2(model_cfg, checkpoint, device=self.device)
        self.predictor = SAM2ImagePredictor(sam2_model)

        # --- 3. ROS INFRASTRUCTURE ---
        self.bridge = CvBridge()
        self.img_color = None
        self.img_depth = None
        self.data_synced = False # Synchronization latch
        
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Image, '/husky_camera/image', self.color_cb, 10)
        self.create_subscription(Image, '/husky_camera/depth_image', self.depth_cb, 10)

        self.is_latched = False

    def color_cb(self, msg):
        self.img_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.data_synced = True # Mark that we have new visual data

    def depth_cb(self, msg):
        self.img_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def get_color_seed(self):
        """Finds the blue cube pixel to prompt SAM2."""
        if self.img_color is None: return None
        hsv = cv2.cvtColor(self.img_color, cv2.COLOR_BGR2HSV)
        # Robust blue range for simulation
        mask = cv2.inRange(hsv, np.array([100, 150, 50]), np.array([140, 255, 255]))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts: return None
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < 100: return None
        M = cv2.moments(c)
        return [int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])]

    def run_perception(self):
        """Processes SAM2 on GPU with dynamic seeding."""
        seed_pixel = self.get_color_seed()
        if seed_pixel is None or self.img_depth is None:
            return None

        # GPU-Accelerated Inference
        img_rgb = cv2.cvtColor(self.img_color, cv2.COLOR_BGR2RGB)
        with torch.inference_mode(), torch.autocast("cuda"):
            self.predictor.set_image(img_rgb)
            masks, _, _ = self.predictor.predict(
                point_coords=np.array([seed_pixel]),
                point_labels=np.array([1]),
                multimask_output=False
            )
        
        mask = masks[0].astype(np.uint8)
        ys, xs = np.where(mask > 0)
        z_vals = self.img_depth[ys, xs]
        
        # Filter for validity
        valid_z = z_vals[np.isfinite(z_vals) & (z_vals > 0.05)]
        if len(valid_z) == 0: return None

        return np.median(valid_z), np.mean((xs - 320) * z_vals / 454.68)

    def run_control(self):
        if self.is_latched:
            self.vel_pub.publish(Twist())
            return

        # 1. WAIT FOR SYNC: Only proceed if data is fresh
        if not self.data_synced:
            return
        
        self.data_synced = False # Consume the data
        self.vel_pub.publish(Twist()) # Hard Stop to process
        
        result = self.run_perception()
        if result is None: return

        dist, err_y = result
        self.get_logger().info(f"SYNCHRONIZED DIST: {dist:.3f}m")

        # 2. STOPPING CRITERIA
        if dist <= self.GRASP_OFFSET:
            self.get_logger().info("!!! OFFSET REACHED. LATCHED !!!")
            self.is_latched = True
            self.vel_pub.publish(Twist())
        else:
            # 3. MOTION BURST
            cmd = Twist()
            cmd.linear.x = 0.15 
            cmd.angular.z = 1.0 * err_y
            self.vel_pub.publish(cmd)
            time.sleep(0.3) 
            self.vel_pub.publish(Twist())

def main():
    rclpy.init()
    node = GpuSyncedReach()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        node.run_control()
    rclpy.shutdown()

if __name__ == '__main__':
    main()