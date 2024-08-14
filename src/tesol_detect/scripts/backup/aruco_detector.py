#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

class ArucoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)
        self.bridge = CvBridge()
        self.node_name = rospy.get_name()
        # Get the ROS parameters
        self.camera_name = rospy.get_param('~camera_name', 'sony_cam2')
        aruco_dict_name = rospy.get_param('~aruco_dict', aruco.DICT_4X4_1000)
        self.aruco_marker_size = rospy.get_param('~aruco_marker_size', 0.02)
        self.experiment_name = rospy.get_param('~experiment_name', 'Exp1')
        # Initialize the subscribers and publishers
        self.image_sub = rospy.Subscriber(f'/{self.camera_name}/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(f'/{self.camera_name}/camera_info', CameraInfo, self.camera_info_callback)
        self.pose_pub = rospy.Publisher(f'/{self.camera_name}_pose/aruco_pose', FiducialTransformArray, queue_size=10)
        # Initialize the camera matrix and distortion coefficients
        self.camera_matrix = None
        self.dist_coeffs = None
        # Setup the aruco dictionary and parameters
        self.aruco_dict = aruco_dict_name
        self.aruco_params = self.setup_aruco_params()
        # Flags to check the camera matrix and distortion coefficients
        self.camera_matrix_received = False
        self.dist_coeffs_received = False
        # Flags to check the recepiton of image and camera info
        self.image_received = False
        self.camera_info_received = False
        # Register with shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        # Initialize the flag for fixed (first) rotation matrix
        self.initial_rotation_matrix = None
    def camera_info_callback(self, msg):
        try:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            self.camera_matrix_received = True
        except Exception as e:
            rospy.logerr(f'Error in camera_info_callback: {e}')
            self.camera_matrix_received = False
    def image_callback(self, msg):
        if self.camera_matrix_received is None or self.dist_coeffs_received is None:
            rospy.logwarn('Camera matrix or distortion coefficients not received')
            return
        else:
            rospy.loginfo('Camera matrix and distortion coefficients received, processing image')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            rospy.loginfo('Image received')
        except CvBridgeError as e:
            rospy.logerr(f'CvBridgeError: {e}')
        else:
            rospy.loginfo('Processing image')
            self.process_image(cv_image)
    def process_image(self, cv_image):
        """ Process the image to detect the aruco markers and publish the pose in the world frame using cornerSubPix accuracy """
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is not None:
            
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.aruco_marker_size, self.camera_matrix, self.dist_coeffs)
            for i in range(len(ids)):
                rospy.loginfo(f'Total Aruco markers detected in the image: {len(ids)}')
                rvec_cur_mar = rvecs[i]
                tvec_cur_mar = tvecs[i]
                aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                aruco.drawDetectedMarkers(cv_image, corners)
                pose = FiducialTransformArray()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = f'aruco_pose_cam{self.camera_name[-1]}'
                for rvec, tvec, fid in zip(rvec_cur_mar, tvec_cur_mar, corners, ids):
                    pose_msg = FiducialTransform()
                    pose_msg.fiducial_id = int(fid)
                    if self.initial_rotation_matrix is None:
                        # initialze  the initial rotation matrix using rodriques formula
                        self.initial_rotation_matrix = cv2.Rodrigues(rvec_cur_mar)[0]
                        print(f'Contents of the initial rotation matrix: {self.initial_rotation_matrix}')
                        print(f'Initial rotation matrix shape: {self.initial_rotation_matrix.shape}')
                        print(f'contents of self.rvec_cur_mar: {rvec_cur_mar}')
                        print(f'Contents of self.tvec_cur_mar: {tvec_cur_mar}')
                        print(f'self.rvec_cur_mar shape: {rvec_cur_mar.shape}')
                    else:
                        # use the initial rotation matrix conver the pose to the world frame by using inverse of initial rotation matrix
                        return
                    pose_msg.transform.translation.x = tvec[0]
                    pose_msg.transform.translation.y = tvec[1]
                    pose_msg.transform.translation.z = tvec[2]
                    pose_msg.transform.rotation.x = 0
                    pose_msg.transform.rotation.y = 0
                    pose_msg.transform.rotation.z = 0
                    pose_msg.transform.rotation.w = 1
                    pose.transforms.append(pose_msg)
                self.pose_pub.publish(pose)
                rospy.loginfo('Aruco pose published')
        else:
            rospy.logwarn('No Aruco markers detected')
        cv2.imshow('Aruco detector', cv_image)
        cv2.waitKey(1)

    def setup_aruco_params(self):
        aruco_params = aruco.DetectorParameters_create()
        aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        aruco_params.cornerRefinementWinSize = 5
        aruco_params.cornerRefinementMaxIterations = 30
        aruco_params.cornerRefinementMinAccuracy = 0.1
        aruco_params.errorCorrectionRate = 0.6
        aruco_params.adaptiveThreshWinSizeMin = 3
        aruco_params.adaptiveThreshWinSizeStep = 1
        aruco_params.adaptiveThreshConstant = 10
        aruco_params.minMarkerPerimeterRate = 0.1
        aruco_params.maxMarkerPerimeterRate = 4.0
        aruco_params.polygonalApproxAccuracyRate = 0.1
        return aruco_params
    def shutdown_hook(self):
        cv2.destroyAllWindows()
        rospy.loginfo('Shutting down Aruco detector node')

if __name__ == '__main__':
    aruco_detector = ArucoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down Aruco detector node')
        cv2.destroyAllWindows()
        

