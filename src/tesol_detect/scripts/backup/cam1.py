#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import cv2.aruco as aruco
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from scipy.spatial.transform import Rotation as R
from filterpy.kalman import KalmanFilter


class MyDetector:
    def __init__(self):
        rospy.init_node("sony_cam1_detect", anonymous=False)
        self.bridge = CvBridge()
        self.node_name = rospy.get_name()
        
        # ROS parameters
        self.camera_name = rospy.get_param("~camera_name", "sony_cam1")
        self.aruco_dict_name = rospy.get_param("~aruco_dict", "DICT_4X4_100")
        self.aruco_marker_size = rospy.get_param("~aruco_marker_size", 0.02)
        self.visualize = rospy.get_param("~visualize", True)
        self.corner_refine_criteria = rospy.get_param(
            "~corner_refine_criteria", 
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.001)
        )
        
        # Subscribers and publishers
        self.image_sub = rospy.Subscriber(f"/{self.camera_name}/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(f"/{self.camera_name}/camera_info", CameraInfo, self.camera_info_callback)
        self.pose_fid_pub = rospy.Publisher(f"/{self.camera_name}/aruco_detect_node/fiducial_transforms", FiducialTransformArray, queue_size=10)
        
        self.aruco_dict = aruco.Dictionary_get(getattr(aruco, self.aruco_dict_name, aruco.DICT_4X4_50))
        self.parameters = aruco.DetectorParameters_create()
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

        rospy.loginfo("Aruco detector node is now running")
        rospy.spin()

    def cleanup(self):
        rospy.loginfo("Shutting down aruco detector node")
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
        """ Process the image message to detect aruco markers """
        input_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        input_image = cv2.resize(input_image, (640, 480))  # Example resizing for faster processing
        gray = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        if ids is not None:
            # Refine the corner locations
            for corner in corners:
                cv2.cornerSubPix(gray, corner, winSize=(5, 5), zeroZone=(-1, -1), criteria=self.corner_refine_criteria)
            
            self.publish_fiducial_transforms(ids, corners, input_image)
        
        if self.visualize:
            aruco.drawDetectedMarkers(input_image, corners, ids)
            cv2.imshow(f"Camera {self.camera_name[-1]}", input_image)
            cv2.waitKey(1)

    def publish_fiducial_transforms(self, ids, corners, image):
        """ Helper function to publish fiducial transforms """
        fiducial_array_msg = FiducialTransformArray()
        fiducial_array_msg.header.stamp = rospy.Time.now()
        fiducial_array_msg.header.frame_id = self.camera_name

        # Estimate pose using estimatePoseSingleMarkers
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, self.aruco_marker_size, self.camera_matrix, self.dist_coeffs
        )

        for i in range(len(ids)):
            # Apply the Kalman Filter for Translation
            measured_tvec = tvecs[i].flatten()
            self.kf_t.predict()
            self.kf_t.update(measured_tvec)
            tvecs[i] = self.kf_t.x[:3].reshape(1, 3)  # Update the translation vector

            # Convert rvec to quaternion for filtering
            rotation_mat, _ = cv2.Rodrigues(rvecs[i])
            measured_quat = R.from_matrix(rotation_mat).as_quat()  # Quaternion [x, y, z, w]

            # Apply the Kalman Filter for Quaternion
            self.kf_q.predict()
            self.kf_q.update(measured_quat.flatten())  # Flatten the quaternion to ensure it is a 1D array
            estimated_quat = self.kf_q.x[:4].flatten()  # Updated quaternion

            # Convert quaternion back to rotation vector
            estimated_rotation = R.from_quat(estimated_quat).as_rotvec().reshape(1, 3)
            rvecs[i] = estimated_rotation

            # Draw and publish the pose
            aruco.drawAxis(image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
            transform = FiducialTransform()
            transform.fiducial_id = int(ids[i])
            transform.transform.translation.x = tvecs[i][0][0]
            transform.transform.translation.y = tvecs[i][0][1]
            transform.transform.translation.z = tvecs[i][0][2]
            transform.transform.rotation.x = estimated_quat[0]
            transform.transform.rotation.y = estimated_quat[1]
            transform.transform.rotation.z = estimated_quat[2]
            transform.transform.rotation.w = estimated_quat[3]
            fiducial_array_msg.transforms.append(transform)
        
        self.pose_fid_pub.publish(fiducial_array_msg)
        # rospy.loginfo(f"Published {len(ids)} fiducial transforms")

if __name__ == "__main__":
    try:
        detector = MyDetector()
    except rospy.ROSInterruptException:
        pass
