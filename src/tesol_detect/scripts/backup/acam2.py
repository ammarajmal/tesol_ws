#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import cv2.aruco as aruco
import apriltag  # Import AprilTag library
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from scipy.spatial.transform import Rotation as R
from filterpy.kalman import KalmanFilter

class MyDetector:
    def __init__(self):
        rospy.init_node("sony_cam2_detect", anonymous=False)
        self.bridge = CvBridge()
        self.node_name = rospy.get_name()
        
        # ROS parameters
        self.camera_name = rospy.get_param("~camera_name", "sony_cam2")
        self.apriltag_family = "tag36h11"  # Change to a supported tag family
        self.aruco_marker_size = rospy.get_param("~aruco_marker_size", 0.020)  # Marker size in meters (20 mm)
        self.visualize = rospy.get_param("~visualize", True)
        
        # Initialize AprilTag detector with the correct family
        self.tag_detector = apriltag.Detector(
            apriltag.DetectorOptions(families=self.apriltag_family)
        )
        
        # Subscribers and publishers
        self.image_sub = rospy.Subscriber(f"/{self.camera_name}/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(f"/{self.camera_name}/camera_info", CameraInfo, self.camera_info_callback)
        self.pose_fid_pub = rospy.Publisher(f"/{self.camera_name}/aruco_detect_node/fiducial_transforms_apt3611", FiducialTransformArray, queue_size=10)
        

        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Kalman Filter for Translation (Position)
        self.kf_t = KalmanFilter(dim_x=6, dim_z=3)
        self.kf_t.F = np.eye(6)  # State transition matrix for translation
        self.kf_t.H = np.hstack([np.eye(3), np.zeros((3, 3))])  # Measurement function for translation
        self.kf_t.P *= 1000.  # Initial uncertainty for translation
        self.kf_t.R = np.eye(3) * 0.01  # Measurement noise for translation
        self.kf_t.Q = np.eye(6) * 0.1  # Process noise for translation
        
        # Kalman Filter for Rotation (Quaternion)
        self.kf_q = KalmanFilter(dim_x=7, dim_z=4)  # Quaternion has 4 components
        self.kf_q.F = np.eye(7)  # State transition matrix for quaternion
        self.kf_q.H = np.hstack([np.eye(4), np.zeros((4, 3))])  # Measurement function for quaternion
        self.kf_q.P *= 1000.  # Initial uncertainty for quaternion
        self.kf_q.R = np.eye(4) * 0.01  # Measurement noise for quaternion
        self.kf_q.Q = np.eye(7) * 0.1  # Process noise for quaternion
        
        self.initial_rotation_mat = None  # To store the initial rotation matrix

        rospy.loginfo("AprilTag detector node is now running")
        rospy.spin()

    def cleanup(self):
        rospy.loginfo("Shutting down AprilTag detector node")
        cv2.destroyAllWindows()

    def camera_info_callback(self, msg):
        """ Callback function for camera info: populate camera matrix and distortion coefficients """
        try:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
        except Exception as e:
            rospy.logerr(e)

    def image_callback(self, msg):
        """ Callback function for image processing: reads image from the camera """
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logwarn("Camera info not received")
            return

        try:
            self.process_img_msg(msg)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge error: {e}")

    def process_img_msg(self, msg):
        """ Process the image message to detect AprilTags """
        input_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        input_image = cv2.resize(input_image, (640, 480))  # Example resizing for faster processing
        gray = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
        
        # Detect AprilTags
        tags = self.tag_detector.detect(gray)
        
        if tags:
            self.publish_fiducial_transforms(tags, input_image)
        
        if self.visualize:
            for tag in tags:
                # Draw the bounding box
                for idx in range(4):
                    # Convert the floating-point coordinates to integers
                    start_point = tuple(tag.corners[idx-1, :].astype(int))
                    end_point = tuple(tag.corners[idx, :].astype(int))
                    cv2.line(input_image, start_point, end_point, (0, 255, 0), 2)
                # Draw the center of the tag
                center_point = tuple(tag.center.astype(int))
                cv2.circle(input_image, center_point, 5, (0, 0, 255), -1)
            cv2.imshow(f"Camera {self.camera_name[-1]}", input_image)
            cv2.waitKey(1)

    def publish_fiducial_transforms(self, tags, image):
        """ Helper function to publish fiducial transforms in the marker frame """
        fiducial_array_msg = FiducialTransformArray()
        fiducial_array_msg.header.stamp = rospy.Time.now()
        fiducial_array_msg.header.frame_id = 'Tag_{}'.format(int(tags[0].tag_id))

        for tag in tags:
            # Estimate pose based on tag corners
            rvec, tvec = self.estimate_tag_pose(tag)

            # Convert rvec to rotation matrix
            rotation_mat, _ = cv2.Rodrigues(rvec)

            if self.initial_rotation_mat is None:
                # Store the initial rotation matrix
                self.initial_rotation_mat = rotation_mat
            
            # Use the initial rotation matrix for inversion
            inv_rotation_mat = np.linalg.inv(self.initial_rotation_mat)
            inv_tvec = -inv_rotation_mat.dot(tvec.flatten())

            # Apply the Kalman Filter for Translation
            self.kf_t.predict()
            self.kf_t.update(inv_tvec)
            inv_tvec = self.kf_t.x[:3].reshape(1, 3)  # Update the translation vector

            # Convert rvec to quaternion for filtering
            measured_quat = R.from_matrix(rotation_mat).as_quat()  # Quaternion [x, y, z, w]

            # Apply the Kalman Filter for Quaternion
            self.kf_q.predict()
            self.kf_q.update(measured_quat.flatten())  # Flatten the quaternion to ensure it is a 1D array
            estimated_quat = self.kf_q.x[:4].flatten()  # Updated quaternion

            # Convert inverted rotation matrix to quaternion
            inv_quat = R.from_quat(estimated_quat).as_quat()

            # Increase axis length for better visibility
            axis_length = 0.2

            # Draw and publish the pose using the initial rotation matrix for inversion
            aruco.drawAxis(image, self.camera_matrix, self.dist_coeffs, inv_rotation_mat, inv_tvec.flatten(), axis_length)
            transform = FiducialTransform()
            transform.fiducial_id = int(tag.tag_id)
            transform.transform.translation.x = inv_tvec[0][0]
            transform.transform.translation.y = inv_tvec[0][1]
            transform.transform.translation.z = inv_tvec[0][2]
            transform.transform.rotation.x = inv_quat[0]
            transform.transform.rotation.y = inv_quat[1]
            transform.transform.rotation.z = inv_quat[2]
            transform.transform.rotation.w = inv_quat[3]
            fiducial_array_msg.transforms.append(transform)
        
        self.pose_fid_pub.publish(fiducial_array_msg)

    def estimate_tag_pose(self, tag):
        """ Estimate the pose of the detected AprilTag """
        # Define the 3D points of the tag in its own coordinate system
        object_points = np.array([
            [-self.aruco_marker_size/2, -self.aruco_marker_size/2, 0],
            [ self.aruco_marker_size/2, -self.aruco_marker_size/2, 0],
            [ self.aruco_marker_size/2,  self.aruco_marker_size/2, 0],
            [-self.aruco_marker_size/2,  self.aruco_marker_size/2, 0]
        ])
        image_points = np.array(tag.corners)

        # SolvePnP to get rvec and tvec
        success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs)
        if not success:
            raise ValueError("Pose estimation failed.")
        return rvec, tvec

if __name__ == "__main__":
    try:
        detector = MyDetector()
    except rospy.ROSInterruptException:
        pass
