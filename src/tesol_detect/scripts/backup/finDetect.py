import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import cv2.aruco as aruco
from cv2.aruco import detectMarkers, estimatePoseSingleMarkers
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray


class myDetector:
    def __init__(self):
        rospy.init_node("aruco_detector", anonymous=False)
        self.bridge = CvBridge()
        self.node_name = rospy.get_name()
        # Ros parameters
        self.camera_name = rospy.get_param("~camera_name", "sony_cam3")
        self.aruco_dict_name = rospy.get_param("~aruco_dict", "DICT_4X4_1000")
        self.aruco_marker_size = rospy.get_param("~aruco_marker_size", 0.02)
        self.experiment_name = rospy.get_param("~experiment_name", "Exp1")
        # Subscribers and publishers
        self.image_sub = rospy.Subscriber(f"/{self.camera_name}/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(f"/{self.camera_name}/camera_info", CameraInfo, self.camera_info_callback)
        # self.pose_pub = rospy.Publisher(f"/{self.camera_name}/aruco_detect/pose", PoseStamped, queue_size=10)
        self.pose_fid_pub = rospy.Publisher(f"/{self.camera_name}/aruco_detect_node/fiducial_transforms", FiducialTransformArray, queue_size=10)
        
        self.camera_matrix = None
        self.dist_coeffs = None
        
        self.aruco_dict = aruco.Dictionary_get(getattr(aruco, self.aruco_dict_name, aruco.DICT_4X4_50))
        self.parameters = aruco.DetectorParameters_create()
        
        self.camera_matrix = None
        self.dist_coeffs = None
        self.pose = PoseStamped()
        self.image_received = False
        self.camera_info_received = False
        
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("Aruco detector node is now running")
        rospy.spin()
    def cleanup(self):
        rospy.loginfo("Shutting down aruco detector node")
        cv2.destroyAllWindows()
    def camera_info_callback(self, msg):
        try:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            self.camera_info_received = True
        except Exception as e:
            rospy.logerr(e)
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
    def image_callback(self, msg):
        if not self.camera_info_received:
            rospy.logwarn("Camera info not received")
            return
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received = True
            gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            aruco.drawDetectedMarkers(self.image, corners, ids)
            if ids is not None:
                for corner in corners:
                    cv2.cornerSubPix(gray, corner, winSize=(10, 10), zeroZone=(-1, -1), criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001))
                rvecs, tvecs, _ = estimatePoseSingleMarkers(corners, self.aruco_marker_size, self.camera_matrix, self.dist_coeffs)
                fiducial_array_msg = FiducialTransformArray()
                fiducial_array_msg.header.stamp = rospy.Time.now()
                fiducial_array_msg.header.frame_id = self.camera_name
                
                for i in range(len(ids)):
                    aruco.drawAxis(self.image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.2)
                    transform = FiducialTransform()
                    transform.fiducial_id = int(ids[i])
                    transform.transform.translation.x = tvecs[i][0][0]
                    transform.transform.translation.y = tvecs[i][0][1]
                    transform.transform.translation.z = tvecs[i][0][2]
                    rotation_mat, _ = cv2.Rodrigues(rvecs[i])
                    quaternion = self.rotation_matrix_to_quaternion_custom(rotation_mat)
                    transform.transform.rotation.w = quaternion[3]  # Access the fourth element (w)
                    transform.transform.rotation.x = quaternion[0]
                    transform.transform.rotation.y = quaternion[1]
                    transform.transform.rotation.z = quaternion[2]
                    transform.fiducial_area = 0.0
                    transform.image_error = 0.0
                    transform.object_error = 0.0
                    
                    # transform.transform.rotation.w = quaternion[3]
                    fiducial_array_msg.transforms.append(transform)
                    print(f'Fiducial msg: {fiducial_array_msg}')
                
                
                    self.pose.header.stamp = msg.header.stamp
                    self.pose.header.frame_id = f"aruco_{ids[i]}"
                    self.pose.pose.position.x = tvecs[i][0][0]
                    self.pose.pose.position.y = tvecs[i][0][1]
                    self.pose.pose.position.z = tvecs[i][0][2]
                    self.pose.pose.orientation.x = quaternion[0]
                    self.pose.pose.orientation.y = quaternion[1]
                    self.pose.pose.orientation.z = quaternion[2]
                    self.pose.pose.orientation.w = quaternion[3]
                    self.pose_fid_pub.publish(fiducial_array_msg)
                    # self.pose_pub.publish(self.pose)
                    # print(f"Published pose for aruco_{ids[i]}")
                    # print(f"tvecs: {tvecs[i]}")
                    # print(f"rvecs: {rvecs[i]}")
            cv2.imshow("aruco_detect", self.image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr(e)
if __name__ == "__main__":
    myDetector()