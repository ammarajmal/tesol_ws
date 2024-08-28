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
        self.corner_refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.001)
        
        # Subscribers and publishers
        self.image_sub = rospy.Subscriber(f"/{self.camera_name}/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(f"/{self.camera_name}/camera_info", CameraInfo, self.camera_info_callback)
        self.pose_fid_pub = rospy.Publisher(f"/{self.camera_name}/aruco_detect_node/fiducial_transforms", FiducialTransformArray, queue_size=10)
        
        self.aruco_dict = aruco.Dictionary_get(getattr(aruco, self.aruco_dict_name, aruco.DICT_4X4_50))
        self.parameters = aruco.DetectorParameters_create()
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # For initial stabilization
        self.initial_rotation_matrices = []
        self.initial_translation_vectors = []
        self.stabilization_frames = 20  # Number of frames to average for stabilization
        self.initial_rotation_matrix = None
        self.initial_translation_vector = None
        
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
        gray = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        if ids is not None:
            # Refine the corner locations
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

                if len(self.initial_rotation_matrices) < self.stabilization_frames:
                    # Accumulate initial rotation matrices and translation vectors
                    self.initial_rotation_matrices.append(rotation_mat)
                    self.initial_translation_vectors.append(tvecs[i])

                if len(self.initial_rotation_matrices) == self.stabilization_frames:
                    # Compute the average rotation matrix and translation vector
                    self.initial_rotation_matrix = self.average_rotation_matrices(self.initial_rotation_matrices)
                    self.initial_translation_vector = np.mean(self.initial_translation_vectors, axis=0)

                if self.initial_rotation_matrix is not None:
                    transform = self.compute_fiducial_transform(int(fid_id), rotation_mat, tvecs[i])
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

    def average_rotation_matrices(self, rotation_matrices):
        """ Averages a list of rotation matrices using quaternions """
        quaternions = [R.from_matrix(mat).as_quat() for mat in rotation_matrices]
        avg_quat = np.mean(quaternions, axis=0)
        avg_quat /= np.linalg.norm(avg_quat)  # Normalize the quaternion
        return R.from_quat(avg_quat).as_matrix()

    def compute_fiducial_transform(self, fiducial_id, rotation_mat, tvec):
        """ Computes the fiducial transform relative to the initial average pose """
        # Ensure tvec and initial_translation_vector are column vectors
        tvec = tvec.reshape(3, 1)
        self.initial_translation_vector = self.initial_translation_vector.reshape(3, 1)
        
        # Compute the relative rotation and translation
        relative_rotation_mat = np.dot(np.linalg.inv(self.initial_rotation_matrix), rotation_mat)
        relative_translation = np.dot(np.linalg.inv(self.initial_rotation_matrix), (tvec - self.initial_translation_vector))

        r_relative = R.from_matrix(relative_rotation_mat)
        quat_relative = r_relative.as_quat()

        transform = FiducialTransform()
        transform.fiducial_id = fiducial_id
        transform.transform.translation.x = relative_translation[0]
        transform.transform.translation.y = relative_translation[1]
        transform.transform.translation.z = relative_translation[2]
        transform.transform.rotation.x = quat_relative[0]
        transform.transform.rotation.y = quat_relative[1]
        transform.transform.rotation.z = quat_relative[2]
        transform.transform.rotation.w = quat_relative[3]

        return transform

if __name__ == "__main__":
    try:
        rospy.init_node("aruco_detector_node", anonymous=False, log_level=rospy.DEBUG)
        detector = MyDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
