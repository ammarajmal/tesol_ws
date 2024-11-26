#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import apriltag
import yaml
import cv2
# Known parameters
W_actual = 40  # Actual marker size in mm
Z = 2400  # Distance from camera to marker in mm
f_initial = 9000  # Initial focal length from camera parameters
image_width = 640  # Image width in pixels
image_height = 480  # Image height in pixels

# ROS topic for the camera image
IMAGE_TOPIC = "/sony_cam2/image_raw"

# Function to calculate marker size in pixels
def calculate_marker_size_pixels(corner_top_left, corner_top_right):
    return np.linalg.norm(np.array(corner_top_right) - np.array(corner_top_left))

# Function to optimize focal length
def optimize_focal_length(W_actual, w_measured, Z, f_initial, tolerance=1e-3, max_iters=100):
    f = f_initial  # Start with the initial focal length
    for i in range(max_iters):
        # Calculate the measured marker size in mm
        W_measured = (w_measured * Z) / f

        # Calculate the error
        error = abs(W_actual - W_measured)
        rospy.loginfo(f"Iteration {i + 1}: Focal length: {f}, Measured size: {W_measured:.3f} mm, Error: {error:.3f} mm")

        # If the error is within the acceptable range, stop optimization
        if error <= tolerance:
            rospy.loginfo("Optimization complete: Error within tolerance.")
            break

        # Adjust the focal length (simple scaling based on the error)
        f = f * (W_measured / W_actual)

    return f

# Function to detect AprilTags and extract corner coordinates
def detect_apriltag_corners(image):
    # Initialize AprilTag detector
    detector = apriltag.Detector()
    
    # Convert to grayscale (AprilTag detection works on grayscale images)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags
    detections = detector.detect(gray)

    if len(detections) == 0:
        raise ValueError("No AprilTags detected in the image.")

    # Assume the first detected tag is the one we want
    tag = detections[0]
    corners = tag.corners  # Get the corners in the order: lb, rb, rt, lt

    # Extract top-left and top-right corners
    corner_top_left = corners[0]  # Top-left corner
    corner_top_right = corners[1]  # Top-right corner

    return corner_top_left, corner_top_right

# ROS node callback to process images
class AprilTagFocalLengthOptimizer:
    def __init__(self):
        rospy.init_node('apriltag_focal_length_optimizer', anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber(IMAGE_TOPIC, Image, self.image_callback)
        self.f_optimized = None
        self.optimization_done = False

    def image_callback(self, msg):
        # Skip processing if optimization is already done
        if self.optimization_done:
            return

        try:
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Step 1: Detect corners of the AprilTag
            corner_top_left, corner_top_right = detect_apriltag_corners(cv_image)
            rospy.loginfo(f"Top-left corner: {corner_top_left}, Top-right corner: {corner_top_right}")

            # Step 2: Calculate the measured marker size in pixels
            w_measured = calculate_marker_size_pixels(corner_top_left, corner_top_right)
            rospy.loginfo(f"Measured marker size in pixels: {w_measured}")

            # Step 3: Optimize the focal length
            self.f_optimized = optimize_focal_length(W_actual, w_measured, Z, f_initial)
            rospy.loginfo(f"Optimized focal length: {self.f_optimized}")

            # Step 4: Save updated camera parameters
            self.save_camera_parameters()

            # Mark optimization as done
            self.optimization_done = True
            rospy.loginfo("Optimization completed. Shutting down node.")
            rospy.signal_shutdown("Optimization completed")

        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
        except ValueError as e:
            rospy.logwarn(f"AprilTag Detection Error: {e}")

    def save_camera_parameters(self):
        camera_matrix = np.array([
            [self.f_optimized, 0, image_width / 2],
            [0, self.f_optimized, image_height / 2],
            [0, 0, 1]
        ])
        rospy.loginfo("Updated camera intrinsic matrix:")
        rospy.loginfo(camera_matrix)

        camera_params = {
            "image_width": image_width,
            "image_height": image_height,
            "distortion_model": "rational_polynomial",
            "camera_name": "sony_cam2",
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": camera_matrix.flatten().tolist(),
            },
            "distortion_coefficients": {
                "rows": 1,
                "cols": 14,
                "data": [-1.56488, -0.0128634, 0.00610089, 0.0116958, -2.16654e-05, -0.714539,
                         0.0128627, 2.16617e-05, 0, 0, 0, 0, 0, 0],  # Replace with actual values
            },
            "rectification_matrix": {
                "rows": 3,
                "cols": 3,
                "data": [1, 0, 0, 0, 1, 0, 0, 0, 1],
            },
            "projection_matrix": {
                "rows": 3,
                "cols": 4,
                "data": [self.f_optimized, 0, image_width / 2, 0,
                         0, self.f_optimized, image_height / 2, 0,
                         0, 0, 1, 0],
            },
        }

        output_file = "optimized_camera_params.yaml"
        with open(output_file, "w") as file:
            yaml.dump(camera_params, file, default_flow_style=False)

        rospy.loginfo(f"Updated camera parameters saved to {output_file}")

# Main entry point
if __name__ == "__main__":
    optimizer = AprilTagFocalLengthOptimizer()
    rospy.spin()
