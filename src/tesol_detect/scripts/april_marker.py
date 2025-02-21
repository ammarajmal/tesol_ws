#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from pupil_apriltags import Detector

class AprilTagDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.node_name = rospy.get_name()

        # ROS parameters
        self.camera_name = rospy.get_param("~camera_name", "sony_cam1")
        self.tag_family = rospy.get_param("~tag_family", "tag36h11")
        self.tag_size = rospy.get_param("~tag_size", 0.020)
        self.visualize = rospy.get_param("~visualize", True)

        # Initialize stabilization variables
        self.initial_rotation_matrices = []
        self.initial_translation_vectors = []
        self.stabilization_frames = 20
        self.initial_rotation_matrix = None
        self.initial_translation_vector = None

        # AprilTag Detector
        self.detector = Detector(families=self.tag_family)

        # ROS subscribers and publishers
        self.image_sub = rospy.Subscriber(f"/{self.camera_name}/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(f"/{self.camera_name}/camera_info", CameraInfo, self.camera_info_callback)
        self.pose_pub = rospy.Publisher(f"/{self.camera_name}/aruco_detect_node/fiducial_transforms", FiducialTransformArray, queue_size=10)

        self.camera_matrix = None
        self.dist_coeffs = None

        rospy.loginfo("AprilTag detector node is now running")

    def camera_info_callback(self, msg):
        try:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D[:5])  # Ensure only the first 5 distortion coefficients are used
        except Exception as e:
            rospy.logerr(e)

    def image_callback(self, msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logwarn("Camera info not received yet. Skipping frame.")
            return

        # Check if image timestamp is valid
        if msg.header.stamp.to_sec() == 0:
            rospy.logwarn("Received image has an invalid timestamp! Skipping detection.")
            return

        try:
            self.process_img_msg(msg)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge error: {e}")

    def process_img_msg(self, msg):
        input_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        detections = self.detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=self.tag_size)

        if detections:
            self.publish_tag_detections(detections, input_image, msg.header.stamp)

            if self.visualize:
                for detection in detections:
                    (ptA, ptB, ptC, ptD) = detection.corners.astype(int)
                    cv2.line(input_image, tuple(ptA), tuple(ptB), (0, 255, 0), 2)
                    cv2.line(input_image, tuple(ptB), tuple(ptC), (0, 255, 0), 2)
                    cv2.line(input_image, tuple(ptC), tuple(ptD), (0, 255, 0), 2)
                    cv2.line(input_image, tuple(ptD), tuple(ptA), (0, 255, 0), 2)

                    center = tuple(detection.center.astype(int))
                    cv2.circle(input_image, center, 5, (0, 0, 255), -1)

                cv2.imshow(f"Camera {self.camera_name[-1]}", input_image)
                cv2.waitKey(1)
        else:
            rospy.logdebug("No AprilTags detected in the current frame.")

    def publish_tag_detections(self, detections, image, img_timestamp):
        if rospy.is_shutdown():
            return

        fiducial_array_msg = FiducialTransformArray()
        fiducial_array_msg.header.stamp = img_timestamp  # Use image timestamp
        fiducial_array_msg.header.frame_id = f'{self.camera_name}_frame'

        # Compute processing delay
        current_time = rospy.Time.now()
        processing_delay = (current_time - img_timestamp).to_sec()
        rospy.loginfo(f"[{img_timestamp.to_sec()}] Processing AprilTag detections... (Processing delay: {processing_delay:.3f} sec)")

        tag_size = self.tag_size
        object_points = np.array([
            [-tag_size/2, -tag_size/2, 0],
            [ tag_size/2, -tag_size/2, 0],
            [ tag_size/2,  tag_size/2, 0],
            [-tag_size/2,  tag_size/2, 0]
        ])

        for detection in detections:
            image_points = detection.corners.reshape(4, 2)

            success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

            if success:
                rotation_mat, _ = cv2.Rodrigues(rvec)

                # Accumulate initial rotation matrices and translation vectors
                if len(self.initial_rotation_matrices) < self.stabilization_frames:
                    self.initial_rotation_matrices.append(rotation_mat)
                    self.initial_translation_vectors.append(tvec)

                if len(self.initial_rotation_matrices) == self.stabilization_frames:
                    self.initial_rotation_matrix = self.average_rotation_matrices(self.initial_rotation_matrices)
                    self.initial_translation_vector = np.mean(self.initial_translation_vectors, axis=0)

                if self.initial_rotation_matrix is not None:
                    transform = self.compute_fiducial_transform(detection.tag_id, rotation_mat, tvec)
                    fiducial_array_msg.transforms.append(transform)

                    if self.visualize:
                        axis_length = self.tag_size / 2
                        cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, rvec, tvec, axis_length)

                    rospy.loginfo(f"[{img_timestamp.to_sec()}] Detected AprilTag ID: {detection.tag_id}, Position: ({tvec[0]}, {tvec[1]}, {tvec[2]})")

        if fiducial_array_msg.transforms:
            self.pose_pub.publish(fiducial_array_msg)
            rospy.loginfo(f"[{img_timestamp.to_sec()}] Published {len(fiducial_array_msg.transforms)} AprilTag detections.")
        else:
            rospy.logwarn(f"[{img_timestamp.to_sec()}] No AprilTags detected.")

    def average_rotation_matrices(self, rotation_matrices):
        """ Averages a list of rotation matrices using quaternions """
        quaternions = [R.from_matrix(mat).as_quat() for mat in rotation_matrices]
        avg_quat = np.mean(quaternions, axis=0)
        avg_quat /= np.linalg.norm(avg_quat)  # Normalize quaternion
        return R.from_quat(avg_quat).as_matrix()

    def compute_fiducial_transform(self, fiducial_id, rotation_mat, tvec):
        relative_rotation_mat = np.dot(np.linalg.inv(self.initial_rotation_matrix), rotation_mat)
        relative_translation = np.dot(np.linalg.inv(self.initial_rotation_matrix), (tvec - self.initial_translation_vector))

        r_relative = R.from_matrix(relative_rotation_mat)
        quat_relative = r_relative.as_quat()

        transform = FiducialTransform()
        transform.fiducial_id = fiducial_id
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = relative_translation
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = quat_relative

        return transform

if __name__ == "__main__":
    rospy.init_node("april_tag_detector_node", anonymous=False)
    detector = AprilTagDetector()
    rospy.spin()
