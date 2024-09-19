#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import apriltag

class AprilTagDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.node_name = rospy.get_name()

        # ROS parameters
        self.camera_name = rospy.get_param("~camera_name", "sony_cam1")
        self.tag_family = rospy.get_param("~tag_family", "tag36h11")
        self.tag_size = rospy.get_param("~tag_size", 0.020)
        self.visualize = rospy.get_param("~visualize", True)

        self.initial_rotation_matrices = []
        self.initial_translation_vectors = []
        self.stabilization_frames = 20  # Number of frames to average for stabilization
        self.initial_rotation_matrix = None
        self.initial_translation_vector = None

        self.detector = apriltag.Detector(apriltag.DetectorOptions(families=self.tag_family))

        self.image_sub = rospy.Subscriber(f"/{self.camera_name}/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(f"/{self.camera_name}/camera_info", CameraInfo, self.camera_info_callback)
        self.pose_pub = rospy.Publisher(f"/{self.camera_name}/aruco_detect_node/fiducial_transforms", FiducialTransformArray, queue_size=10)

        self.camera_matrix = None
        self.dist_coeffs = None

        rospy.loginfo("AprilTag detector node is now running")

    def camera_info_callback(self, msg):
        try:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
        except Exception as e:
            rospy.logerr(e)

    def image_callback(self, msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logwarn("Camera info not received")
            return

        try:
            self.process_img_msg(msg)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge error: {e}")

    def process_img_msg(self, msg):
        input_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(gray)

        if detections:
            self.publish_tag_detections(detections, input_image)

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

    def publish_tag_detections(self, detections, image):
        if rospy.is_shutdown():
            return

        fiducial_array_msg = FiducialTransformArray()
        fiducial_array_msg.header.stamp = rospy.Time.now()
        fiducial_array_msg.header.frame_id = f'{self.camera_name}_frame'

        tag_size = self.tag_size
        object_points = np.array([
            [-tag_size/2, -tag_size/2, 0],
            [ tag_size/2, -tag_size/2, 0],
            [ tag_size/2,  tag_size/2, 0],
            [-tag_size/2,  tag_size/2, 0]
        ])

        for detection in detections:
            if detection.tag_id == 1:
                image_points = detection.corners.reshape(4, 2)

                success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

                if success:
                    rotation_mat, _ = cv2.Rodrigues(rvec)
                    
                    # Accumulate initial rotation matrices and translation vectors
                    if len(self.initial_rotation_matrices) < self.stabilization_frames:
                        self.initial_rotation_matrices.append(rotation_mat)
                        self.initial_translation_vectors.append(tvec)

                    if len(self.initial_rotation_matrices) == self.stabilization_frames:
                        # Compute the average rotation matrix and translation vector
                        self.initial_rotation_matrix = self.average_rotation_matrices(self.initial_rotation_matrices)
                        self.initial_translation_vector = np.mean(self.initial_translation_vectors, axis=0)

                    if self.initial_rotation_matrix is not None:
                        transform = self.compute_fiducial_transform(detection.tag_id, rotation_mat, tvec)
                        fiducial_array_msg.transforms.append(transform)

                        if self.visualize:
                            axis_length = self.tag_size / 2
                            cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, rvec, tvec, axis_length)

        self.pose_pub.publish(fiducial_array_msg)

    def average_rotation_matrices(self, rotation_matrices):
        """ Averages a list of rotation matrices using quaternions """
        quaternions = [R.from_matrix(mat).as_quat() for mat in rotation_matrices]
        avg_quat = np.mean(quaternions, axis=0)
        avg_quat /= np.linalg.norm(avg_quat)  # Normalize the quaternion
        return R.from_quat(avg_quat).as_matrix()

    def compute_fiducial_transform(self, fiducial_id, rotation_mat, tvec):
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
        rospy.init_node("april_tag_detector_node", anonymous=False, log_level=rospy.DEBUG)
        detector = AprilTagDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
