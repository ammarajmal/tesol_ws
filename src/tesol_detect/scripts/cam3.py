#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import cv2.aruco as aruco
from cv2.aruco import detectMarkers, estimatePoseSingleMarkers
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from scipy.spatial.transform import Rotation as R


class MyDetector:
    def __init__(self):
        rospy.init_node("sony_cam3_detect", anonymous=False)
        self.bridge = CvBridge()
        self.node_name = rospy.get_name()
        
        # ROS parameters
        self.camera_name = rospy.get_param("~camera_name", "sony_cam3")
        self.aruco_dict_name = rospy.get_param("~aruco_dict", "DICT_4X4_100")
        self.aruco_marker_size = rospy.get_param("~aruco_marker_size", 0.025)
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

            # Estimate pose using cv2.estimatePoseSingleMarkers
            rvecs, tvecs, _ = estimatePoseSingleMarkers(
                corners, self.aruco_marker_size, self.camera_matrix, self.dist_coeffs
            )

            for i in range(len(ids)):
                aruco.drawAxis(image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
                transform = FiducialTransform()
                transform.fiducial_id = int(ids[i])
                transform.transform.translation.x = tvecs[i][0][0]
                transform.transform.translation.y = tvecs[i][0][1]
                transform.transform.translation.z = tvecs[i][0][2]
                rotation_mat, _ = cv2.Rodrigues(rvecs[i])
                quaternion = R.from_matrix(rotation_mat).as_quat()
                transform.transform.rotation.w = quaternion[3]
                transform.transform.rotation.x = quaternion[0]
                transform.transform.rotation.y = quaternion[1]
                transform.transform.rotation.z = quaternion[2]
                fiducial_array_msg.transforms.append(transform)
            
            self.pose_fid_pub.publish(fiducial_array_msg)
            # rospy.loginfo(f"Published {len(ids)} fiducial transforms")

if __name__ == "__main__":
    try:
        detector = MyDetector()
    except rospy.ROSInterruptException:
        pass
