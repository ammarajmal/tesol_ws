#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import cv2.aruco as aruco
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from scipy.spatial.transform import Rotation as R

class MyDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.node_name = rospy.get_name()
        
        # ROS parameters
        self.camera_name = rospy.get_param("~camera_name", "sony_cam1")
        self.aruco_dict_name = rospy.get_param("~dictionary", "DICT_7X7_1000")
        self.aruco_marker_size = rospy.get_param("~fiducial_len", 0.020)
        self.visualize = rospy.get_param("~visualize", True)
        self.corner_refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0.001)
        # Subscribers and publishers
        self.image_sub = rospy.Subscriber(f"/{self.camera_name}/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(f"/{self.camera_name}/camera_info", CameraInfo, self.camera_info_callback)
        self.pose_fid_pub = rospy.Publisher(f"/{self.camera_name}/aruco_detect_node/fiducial_transforms", FiducialTransformArray, queue_size=10)
        
        self.aruco_dict = aruco.Dictionary_get(getattr(aruco, self.aruco_dict_name, aruco.DICT_4X4_50))
        self.parameters = aruco.DetectorParameters_create()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.initial_rotation_mats = []
        self.initial_rotation_frames = 10
        self.initial_rotation_mat = None  # To store the initial rotation matrix for conversion of pose from camera to marker frame
        self.last_known_transforms = {}  # To store the last known transforms for each marker ID


        rospy.loginfo("Aruco detector node is now running")

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
        # input_image = cv2.resize(input_image, (640, 480))  # Example resizing for faster processing
        gray = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        # corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)
        
        if ids is not None:
            # # Refine the corner locations
            for corner in corners:
                cv2.cornerSubPix(gray, corner, winSize=(10, 10), zeroZone=(-1, -1), criteria=self.corner_refine_criteria)
            
            self.publish_fiducial_transforms(ids, corners, input_image)
        
            if self.visualize:
                aruco.drawDetectedMarkers(input_image, corners, ids)
                cv2.imshow(f"Camera {self.camera_name[-1]}", input_image)
                cv2.waitKey(1)
        else:
            rospy.logdebug("No markers detected in the current frame.")
        
    def publish_fiducial_transforms(self, ids, corners, image):
        if rospy.is_shutdown():
            return

        fiducial_array_msg = FiducialTransformArray()
        fiducial_array_msg.header.stamp = rospy.Time.now()
        fiducial_array_msg.header.frame_id = f'{self.camera_name}_frame'

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.aruco_marker_size, self.camera_matrix, self.dist_coeffs
            )

            for i, fid_id in enumerate(ids):
                rotation_mat, _ = cv2.Rodrigues(rvecs[i])

                if self.initial_rotation_mat is None:
                    self.initial_rotation_mat = rotation_mat

                inv_rotation_mat = np.linalg.inv(self.initial_rotation_mat)
                inv_tvec = -inv_rotation_mat.dot(tvecs[i].flatten())

                transform = FiducialTransform()
                transform.fiducial_id = int(fid_id)
                transform.transform.translation.x = inv_tvec[0]
                transform.transform.translation.y = inv_tvec[1]
                transform.transform.translation.z = inv_tvec[2]
                r = R.from_matrix(inv_rotation_mat)
                quat = r.as_quat()
                transform.transform.rotation.x = quat[0]
                transform.transform.rotation.y = quat[1]
                transform.transform.rotation.z = quat[2]
                transform.transform.rotation.w = quat[3]

                fiducial_array_msg.transforms.append(transform)
                
                # Update the last known transform
                self.last_known_transforms[int(fid_id)] = transform

                if self.visualize:
                    aruco.drawAxis(image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], self.aruco_marker_size/2)
        else:
            # If no markers are detected, use the last known transforms
            rospy.logdebug("No markers detected, using last known transforms.")
            for fid_id, last_transform in self.last_known_transforms.items():
                fiducial_array_msg.transforms.append(last_transform)

        self.pose_fid_pub.publish(fiducial_array_msg)

if __name__ == "__main__":
    try:
        rospy.init_node("aruco_detector_node", anonymous=False, log_level=rospy.DEBUG)
        detector = MyDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
