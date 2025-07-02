#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from pupil_apriltags import Detector
from collections import deque
from filterpy.kalman import KalmanFilter

class AprilTagDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.node_name = rospy.get_name()

        self.camera_name = rospy.get_param("~camera_name", "sony_cam1")
        self.tag_family  = rospy.get_param("~tag_family", "tag36h11")
        self.tag_size    = rospy.get_param("~tag_size", 0.020)
        self.visualize   = rospy.get_param("~visualize", True)

        self.initial_rotation_matrices   = []
        self.initial_translation_vectors = []
        self.stabilization_frames        = 20
        self.initial_rotation_matrix     = None
        self.initial_translation_vector  = None

        self.kalman_filter_initialized = False
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.MAX_ALLOWED_DISPLACEMENT = 100.0

        self.detector = Detector(
            families=self.tag_family,
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        self.image_sub = rospy.Subscriber(
            f"/{self.camera_name}/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(
            f"/{self.camera_name}/camera_info", CameraInfo, self.camera_info_callback)
        self.pose_pub = rospy.Publisher(
            f"/{self.camera_name}/aruco_detect_node/fiducial_transforms",
            FiducialTransformArray,
            queue_size=10
        )

        self.camera_matrix = None
        self.dist_coeffs   = None

        rospy.loginfo("AprilTag detector node is now running")

    def camera_info_callback(self, msg):
        try:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.D[:5])
        except Exception as e:
            rospy.logerr(e)

    def image_callback(self, msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logwarn("Camera info not received yet. Skipping frame.")
            return

        if msg.header.stamp.to_sec() == 0:
            rospy.logwarn("Invalid timestamp on image. Skipping detection.")
            return

        try:
            self.process_img_msg(msg)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge error: {e}")

    def process_img_msg(self, msg):
        input_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray        = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(
            gray,
            estimate_tag_pose=False,
            camera_params=None,
            tag_size=self.tag_size
        )

        if detections:
            self.publish_tag_detections(detections, gray, input_image, msg.header.stamp)
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

    def publish_tag_detections(self, detections, gray_image, input_image, img_timestamp):
        fiducial_array_msg = FiducialTransformArray()
        fiducial_array_msg.header.stamp    = img_timestamp
        fiducial_array_msg.header.frame_id = f"{self.camera_name}_frame"

        tag_size = self.tag_size
        object_points = np.array([
            [-tag_size/2, -tag_size/2, 0],
            [ tag_size/2, -tag_size/2, 0],
            [ tag_size/2,  tag_size/2, 0],
            [-tag_size/2,  tag_size/2, 0]
        ])

        for detection in detections:
            corners = detection.corners.astype(np.float32)
            refined_corners = cv2.cornerSubPix(
                gray_image,
                corners.reshape(-1, 1, 2),
                (3, 3),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
            )
            image_points = refined_corners.reshape(4, 2)

            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_EPNP,
                reprojectionError=0.1
            )
            if not success:
                continue

            rotation_mat, _ = cv2.Rodrigues(rvec)

            if len(self.initial_rotation_matrices) < self.stabilization_frames:
                self.initial_rotation_matrices.append(rotation_mat)
                self.initial_translation_vectors.append(tvec)
            elif self.initial_rotation_matrix is None:
                self.initial_rotation_matrix    = self.average_rotation_matrices(self.initial_rotation_matrices)
                self.initial_translation_vector = np.mean(self.initial_translation_vectors, axis=0)

            if self.initial_rotation_matrix is not None:
                relative_rotation_mat = np.dot(
                    np.linalg.inv(self.initial_rotation_matrix), rotation_mat)
                relative_translation = np.dot(
                    np.linalg.inv(self.initial_rotation_matrix),
                    (tvec - self.initial_translation_vector))

                stable_position = self.kalman_filter(relative_translation.flatten())
                r_relative = R.from_matrix(relative_rotation_mat)
                quat = r_relative.as_quat()

                transform = FiducialTransform()
                transform.fiducial_id = detection.tag_id
                transform.transform.translation.x = stable_position[0]
                transform.transform.translation.y = stable_position[1]
                transform.transform.translation.z = stable_position[2]
                transform.transform.rotation.x = quat[0]
                transform.transform.rotation.y = quat[1]
                transform.transform.rotation.z = quat[2]
                transform.transform.rotation.w = quat[3]
                fiducial_array_msg.transforms.append(transform)

                if self.visualize:
                    cv2.drawFrameAxes(input_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, tag_size * 0.5)

        if fiducial_array_msg.transforms:
            self.pose_pub.publish(fiducial_array_msg)

    def average_rotation_matrices(self, rotation_matrices):
        quaternions = [R.from_matrix(mat).as_quat() for mat in rotation_matrices]
        avg_quat    = np.mean(quaternions, axis=0)
        avg_quat   /= np.linalg.norm(avg_quat)
        return R.from_quat(avg_quat).as_matrix()

    def kalman_filter(self, position):
        if not self.kalman_filter_initialized:
            self.kf.x = np.hstack((position, [0, 0, 0]))
            self.kf.F = np.array([[1,0,0,1,0,0], [0,1,0,0,1,0], [0,0,1,0,0,1],
                                  [0,0,0,1,0,0], [0,0,0,0,1,0], [0,0,0,0,0,1]])
            self.kf.H = np.array([[1,0,0,0,0,0], [0,1,0,0,0,0], [0,0,1,0,0,0]])
            self.kf.P *= 0.01
            self.kf.R *= 0.001
            self.kf.Q *= 0.001
            self.kalman_filter_initialized = True
        self.kf.predict()
        self.kf.update(position)
        return self.kf.x[:3]

if __name__ == "__main__":
    rospy.init_node("april_tag_detector_node", anonymous=False)
    detector = AprilTagDetector()
    rospy.spin()
