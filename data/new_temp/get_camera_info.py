#!/usr/bin/env python
#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import apriltag
import os
import numpy as np
import yaml

# Global variables
image_count = 0
max_images = 30
bridge = CvBridge()
save_directory = './captured_images/'
camera_matrix = None
dist_coeffs = None
tag_size = 0.02  # 20mm in meters
apriltag_family = "tag36h11"

# Create the directory to save images
if not os.path.exists(save_directory):
    os.makedirs(save_directory)

# 3D corner points of the AprilTag in the tag's coordinate system
tag_corners_3d = np.array([
    [-tag_size / 2, -tag_size / 2, 0],
    [tag_size / 2, -tag_size / 2, 0],
    [tag_size / 2, tag_size / 2, 0],
    [-tag_size / 2, tag_size / 2, 0]
], dtype=np.float32)

obj_points = []  # 3D points in real-world space
img_points = []  # 2D points in image plane

# AprilTag detector
detector = apriltag.Detector(apriltag.DetectorOptions(families=apriltag_family))

# Callback to capture images
def image_callback(msg):
    global image_count
    global camera_matrix, dist_coeffs

    if image_count >= max_images:
        return

    # Convert ROS Image message to OpenCV image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
        return

    # Save the image
    image_filename = os.path.join(save_directory, f'image_{image_count + 1}.png')
    cv2.imwrite(image_filename, cv_image)
    rospy.loginfo(f"Saved image {image_count + 1} to {image_filename}")
    
    # Detect AprilTags
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    results = detector.detect(gray)
    if results:
        for r in results:
            img_corners_2d = np.array(r.corners, dtype=np.float32)

            # Append the 3D and 2D points for calibration refinement
            obj_points.append(tag_corners_3d)
            img_points.append(img_corners_2d)

            # Draw detected tag corners on the image for verification
            for corner in img_corners_2d:
                cv2.circle(cv_image, tuple(corner), 5, (0, 255, 0), -1)

        # Display the image with detected tags
        cv2.imshow("AprilTag Detection", cv_image)
        cv2.waitKey(100)

    # Increment the counter
    image_count += 1

    # Stop after capturing max_images
    if image_count >= max_images:
        rospy.signal_shutdown("Captured 30 images, proceeding with calibration.")

# Callback to get camera parameters
def camera_info_callback(msg):
    global camera_matrix, dist_coeffs
    camera_matrix = np.array(msg.K).reshape(3, 3)
    dist_coeffs = np.array(msg.D)

    rospy.loginfo(f"Camera Matrix: {camera_matrix}")
    rospy.loginfo(f"Distortion Coefficients: {dist_coeffs}")

def capture_and_refine():
    rospy.init_node('image_capture_node', anonymous=True)

    # Subscribe to the camera info and image topic
    rospy.Subscriber('/sony_cam3/camera_info', CameraInfo, camera_info_callback)
    rospy.Subscriber('/sony_cam3/image_raw', Image, image_callback)

    rospy.loginfo("Waiting for images and camera info...")
    rospy.spin()

    if camera_matrix is not None and dist_coeffs is not None:
        rospy.loginfo("Starting camera calibration refinement...")

        # Perform calibration refinement using the captured points
        image_size = (1280, 720)  # Adjust according to your camera
        ret, refined_camera_matrix, refined_dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            obj_points, img_points, image_size, camera_matrix, dist_coeffs, flags=cv2.CALIB_USE_INTRINSIC_GUESS
        )

        # Print and save the refined camera matrix and distortion coefficients
        rospy.loginfo(f"Refined Camera Matrix: \n{refined_camera_matrix}")
        rospy.loginfo(f"Refined Distortion Coefficients: \n{refined_dist_coeffs}")

        # Save the refined camera parameters
        refined_params = {
            'image_width': image_size[0],
            'image_height': image_size[1],
            'camera_matrix': refined_camera_matrix.tolist(),
            'distortion_coefficients': refined_dist_coeffs.tolist()
        }
        with open('refined_camera_params.yaml', 'w') as f:
            yaml.dump(refined_params, f)

        rospy.loginfo("Refined parameters saved to refined_camera_params.yaml")
    else:
        rospy.logerr("Failed to retrieve camera info.")

if __name__ == '__main__':
    try:
        capture_and_refine()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
