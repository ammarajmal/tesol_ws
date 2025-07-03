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
from scipy.signal import savgol_filter

class AprilTagDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.node_name = rospy.get_name()

        # ROS parameters
        self.camera_name = rospy.get_param("~camera_name", "sony_cam1")
        self.tag_family  = rospy.get_param("~tag_family", "tag36h11")
        self.tag_size    = rospy.get_param("~tag_size", 0.020)
        self.visualize   = rospy.get_param("~visualize", True)

        # ---------------------- Stabilization & Filtering ----------------------
        # 1) We'll collect an initial 20 frames to “stabilize” the reference frame
        self.initial_rotation_matrices   = []
        self.initial_translation_vectors = []
        self.stabilization_frames        = 20
        self.initial_rotation_matrix     = None
        self.initial_translation_vector  = None

        # 2) Outlier & smoothing
        self.position_history      = deque(maxlen=10)  # For outlier rejection
        self.displacement_buffer   = deque(maxlen=15)  # For Savitzky-Golay
        self.MAX_ALLOWED_DISPLACEMENT = 100.0          # mm

        # AprilTag Detector
        self.detector = Detector(
            families=self.tag_family,
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # ROS subscribers/publishers
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

    # -------------------------------------------------------------------------
    #                    ROS Callbacks
    # -------------------------------------------------------------------------
    def camera_info_callback(self, msg):
        """Store the camera matrix/distortion once available."""
        try:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.D[:5])  # only first 5
        except Exception as e:
            rospy.logerr(e)

    def image_callback(self, msg):
        """Convert to OpenCV, detect, etc."""
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

    # -------------------------------------------------------------------------
    #                   Main Processing
    # -------------------------------------------------------------------------
    def process_img_msg(self, msg):
        input_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray        = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags (no direct pose from the library)
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=False,
            camera_params=None,
            tag_size=self.tag_size
        )

        if detections:
            # We'll compute the transforms with solvePnP and publish
            self.publish_tag_detections(detections, input_image, msg.header.stamp)
            # Then draw outlines of the marker corners + center
            if self.visualize:
                for detection in detections:
                    (ptA, ptB, ptC, ptD) = detection.corners.astype(int)
                    # Draw green lines
                    cv2.line(input_image, tuple(ptA), tuple(ptB), (0, 255, 0), 2)
                    cv2.line(input_image, tuple(ptB), tuple(ptC), (0, 255, 0), 2)
                    cv2.line(input_image, tuple(ptC), tuple(ptD), (0, 255, 0), 2)
                    cv2.line(input_image, tuple(ptD), tuple(ptA), (0, 255, 0), 2)

                    # Draw center circle
                    center = tuple(detection.center.astype(int))
                    cv2.circle(input_image, center, 5, (0, 0, 255), -1)

                cv2.imshow(f"Camera {self.camera_name[-1]}", input_image)
                cv2.waitKey(1)
        else:
            rospy.logdebug("No AprilTags detected in this frame.")

    def publish_tag_detections(self, detections, input_image, img_timestamp):
        """
        For each detection:
         1) solvePnP => rvec, tvec
         2) Accumulate initial frames for stable reference (20 frames)
         3) Once stable, compute relative transform
         4) Apply outlier rejection + smoothing
         5) Publish & optionally draw 3D axes
        """
        fiducial_array_msg = FiducialTransformArray()
        fiducial_array_msg.header.stamp    = img_timestamp
        fiducial_array_msg.header.frame_id = f"{self.camera_name}_frame"

        # 3D corners of the tag
        tag_size = self.tag_size
        object_points = np.array([
            [-tag_size/2, -tag_size/2, 0],
            [ tag_size/2, -tag_size/2, 0],
            [ tag_size/2,  tag_size/2, 0],
            [-tag_size/2,  tag_size/2, 0]
        ])

        for detection in detections:
            image_points = detection.corners.reshape(4, 2)
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            if not success:
                continue

            # Convert to rotation matrix
            rotation_mat, _ = cv2.Rodrigues(rvec)

            # 1) Accumulate for initial stabilization
            if len(self.initial_rotation_matrices) < self.stabilization_frames:
                self.initial_rotation_matrices.append(rotation_mat)
                self.initial_translation_vectors.append(tvec)
            elif self.initial_rotation_matrix is None:
                # We just reached 20 frames: average them
                self.initial_rotation_matrix    = self.average_rotation_matrices(self.initial_rotation_matrices)
                self.initial_translation_vector = np.mean(self.initial_translation_vectors, axis=0)

            # 2) If stable, compute final transform
            if self.initial_rotation_matrix is not None:
                # Relative rotation from the stabilized reference
                relative_rotation_mat = np.dot(
                    np.linalg.inv(self.initial_rotation_matrix),
                    rotation_mat
                )
                relative_translation = np.dot(
                    np.linalg.inv(self.initial_rotation_matrix),
                    (tvec - self.initial_translation_vector)
                )

                # Convert to Euler for logging
                r_relative  = R.from_matrix(relative_rotation_mat)
                euler_deg   = r_relative.as_euler('xyz', degrees=True)

                # Now let's do outlier + smoothing on the final "relative_translation"
                # Because your outlier thresholds & smoothing are in meter-based logic
                # We'll pass it to correct_outlier / smooth_displacement
                stable_position = self.correct_outlier(relative_translation.flatten())
                # stable_position = self.smooth_displacement(stable_position)

                # Log
                # rospy.loginfo(
                #     f"[{img_timestamp.to_sec():.3f}s] TagID={detection.tag_id} "
                #     f"| Pos=({stable_position[0]:.4f}, {stable_position[1]:.4f}, {stable_position[2]:.4f}) m "
                #     f"| Euler=({euler_deg[0]:.2f}, {euler_deg[1]:.2f}, {euler_deg[2]:.2f}) deg"
                # )

                # Build the transform for publishing
                transform = FiducialTransform()
                transform.fiducial_id = detection.tag_id
                transform.transform.translation.x = stable_position[0]
                transform.transform.translation.y = stable_position[1]
                transform.transform.translation.z = stable_position[2]

                # If you want orientation in the message, we can build a quaternion
                # from r_relative. For now, let's do that:
                quat_relative = r_relative.as_quat()  # x,y,z,w
                (transform.transform.rotation.x,
                 transform.transform.rotation.y,
                 transform.transform.rotation.z,
                 transform.transform.rotation.w) = quat_relative

                fiducial_array_msg.transforms.append(transform)

                # 3) Visualization: draw 3D axes once stable
                if self.visualize:
                    axis_length = tag_size * 0.5
                    cv2.drawFrameAxes(input_image,
                                      self.camera_matrix,
                                      self.dist_coeffs,
                                      rvec,
                                      tvec,
                                      axis_length)

        # If we got any transforms, publish them
        if fiducial_array_msg.transforms:
            self.pose_pub.publish(fiducial_array_msg)

    # -------------------------------------------------------------------------
    #                   Stabilization & Outlier Logic
    # -------------------------------------------------------------------------
    def average_rotation_matrices(self, rotation_matrices):
        """Average rotation matrices by converting each to a quaternion first."""
        quaternions = [R.from_matrix(mat).as_quat() for mat in rotation_matrices]
        avg_quat    = np.mean(quaternions, axis=0)
        avg_quat   /= np.linalg.norm(avg_quat)  # normalize
        return R.from_quat(avg_quat).as_matrix()

    def correct_outlier(self, position):
        """
        If there's a large jump in measured position (in meters),
        we hold the last stable value. We'll compare in mm.
        """
        if len(self.position_history) > 0:
            prev_position = self.position_history[-1]
            displacement_change_mm = np.linalg.norm(position - prev_position) * 1000.0
            if displacement_change_mm > self.MAX_ALLOWED_DISPLACEMENT:
                rospy.logwarn(f"Outlier: jump of {displacement_change_mm:.2f} mm => revert to previous.")
                return prev_position
        self.position_history.append(position)
        return position

    def smooth_displacement(self, position):
        """Apply Savitzky-Golay smoothing on the final, stabilized position."""
        self.displacement_buffer.append(position)
        if len(self.displacement_buffer) >= 5:
            data_array   = np.array(self.displacement_buffer)
            smoothed_data = savgol_filter(data_array, window_length=5, polyorder=2, axis=0)
            return smoothed_data[-1]
        return position

# -------------------------------------------------------------------------
#                       ROS Node Entry Point
# -------------------------------------------------------------------------
if __name__ == "__main__":
    rospy.init_node("april_tag_detector_node", anonymous=False)
    detector = AprilTagDetector()
    rospy.spin()
