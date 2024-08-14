#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from cv_bridge import CvBridge, CvBridgeError
import tf

class arucoDetect:
    def __init__(self):
        rospy.init_node('aruco_detect', anonymous=False)
        
        self.bridge = CvBridge()
        self.node_name = rospy.get_name()

        # Get the ROS parameters
        self.camera_name = rospy.get_param('~camera_name', 'sony_cam2')
        self.aruco_dict_name = rospy.get_param('~aruco_dict', 'DICT_4X4_1000')
        self.aruco_marker_size = rospy.get_param('~aruco_marker_size', 0.02)
        self.experiment_name = rospy.get_param('~experiment_name', 'Exp1')

        # Subscribers and publishers
        self.image_sub = rospy.Subscriber(f'/{self.camera_name}/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(f'/{self.camera_name}/camera_info', CameraInfo, self.camera_info_callback)
        self.pose_pub = rospy.Publisher(f'/{self.camera_name}/aruco_detect_node/fiducial_transforms', FiducialTransformArray, queue_size=10)

        # Initialize the camera matrix, distortion coefficients, aruco dictionary, and aruco parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.aruco_dict = aruco.Dictionary_get(getattr(aruco, self.aruco_dict_name, aruco.DICT_4X4_50))
        self.aruco_params = self.setup_aruco_params()

        self.camera_matrix_received = False
        self.dist_coeffs_received = False
        self.image_received = False
        self.camera_info_received = False

        rospy.on_shutdown(self.shutdown_hook)

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.aruco_pose = FiducialTransformArray()

    def setup_aruco_params(self):
        aruco_params = aruco.DetectorParameters_create()
        return aruco_params

    def camera_info_callback(self, msg):
        try:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            self.camera_matrix_received = True
            self.dist_coeffs_received = True
        except Exception as e:
            rospy.logerr(f'Error in camera_info_callback: {e}')
            self.camera_matrix_received = False

    def image_callback(self, msg):
        if not self.camera_matrix_received or not self.dist_coeffs_received:
            rospy.logwarn('Camera matrix or distortion coefficients not received')
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_received = True
        except CvBridgeError as e:
            rospy.logerr(f'CvBridgeError: {e}')
            return
        if self.image_received and self.camera_matrix_received and self.dist_coeffs_received:
            self.detect_markers(cv_image)

    def detect_markers(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None:
            for corner in corners:
                cv2.cornerSubPix(gray, corner, winSize=(3, 3), zeroZone=(-1, -1), criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            aruco.drawDetectedMarkers(cv_image, corners, ids)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.aruco_marker_size, self.camera_matrix, self.dist_coeffs)
            # prepare the FiducialTransformArray message
            transform_array_msg = FiducialTransformArray()
            transform_array_msg.header.stamp = rospy.Time.now()
            transform_array_msg.header.frame_id = self.camera_name
            for i in range(len(ids)):
                aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
                for rvec, tvec ,corner, fid in zip(rvecs, tvecs, corners, ids):
                    transform = FiducialTransform()
                    transform.fiducial_id = int(fid)
                    transform.transform.translation.x = tvec[0][0]
                    transform.transform.translation.y = tvec[0][1]
                    transform.transform.translation.z = tvec[0][2]
                    q = self.rotation_matrix_to_quaternion_custom(cv2.Rodrigues(rvec)[0])
                    transform.transform.rotation.x = q[0]
                    transform.transform.rotation.y = q[1]
                    transform.transform.rotation.z = q[2]
                    transform.transform.rotation.w = q[3]
                    
                    transform_array_msg.transforms.append(transform)
            self.pose_pub.publish(transform_array_msg)
            cv2.imshow('aruco_detect', cv_image)
            cv2.waitKey(1)
        else:
            rospy.loginfo('No markers detected')

    def rotation_matrix_to_quaternion_custom(self, R):
        q = np.empty((4,))
        t = np.trace(R)
        if t > 0:
            S = np.sqrt(t + 1.0) * 2
            q[3] = 0.25 * S
            q[0] = (R[2, 1] - R[1, 2]) / S
            q[1] = (R[0, 2] - R[2, 0]) / S
            q[2] = (R[1, 0] - R[0, 1]) / S
        else:
            if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
                S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                q[3] = (R[2, 1] - R[1, 2]) / S
                q[0] = 0.25 * S
                q[1] = (R[0, 1] + R[1, 0]) / S
                q[2] = (R[0, 2] + R[2, 0]) / S
            elif (R[1, 1] > R[2, 2]):
                S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                q[3] = (R[0, 2] - R[2, 0]) / S
                q[0] = (R[0, 1] + R[1, 0]) / S
                q[1] = 0.25 * S
                q[2] = (R[1, 2] + R[2, 1]) / S
            else:
                S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                q[3] = (R[1, 0] - R[0, 1]) / S
                q[0] = (R[0, 2] + R[2, 0]) / S
                q[1] = (R[1, 2] + R[2, 1]) / S

        # Normalize quaternion
        q /= np.linalg.norm(q)
        return q

    def shutdown_hook(self):
        cv2.destroyAllWindows()
        rospy.loginfo('Shutting down aruco_detect node')

if __name__ == '__main__':
    try:
        aruco_detect = arucoDetect()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
