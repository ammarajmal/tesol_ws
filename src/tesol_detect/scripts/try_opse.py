import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import cv2.aruco as aruco
from cv2.aruco import detectMarkers, estimatePoseSingleMarkers

# Aruco dictionary and parameters
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
parameters = cv2.aruco.DetectorParameters_create()
camera_matrix = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
camera_info_received = False
image_received = False

# Function to publish marker pose and timestamp
def publish_marker_pose(marker_ids, rvecs, tvecs, camera_matrix, dist_coeffs, timestamp):
    # Create a ROS publisher for marker poses
    pub = rospy.Publisher('aruco_marker_poses', PoseStamped, queue_size=10)

    for i in range(len(marker_ids)):
        # Create a PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()  # Use the provided timestamp if available
        pose_msg.header.frame_id = "camera_link"  # Replace with your camera frame

        # Convert rotation vector to quaternion
        rotation_mat, _ = cv2.Rodrigues(rvecs[i])
        _, _, _, _, _, _, quaternion = cv2.RQDecomp3x3(rotation_mat)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        # Convert translation vector to position
        pose_msg.pose.position.x = tvecs[i][0][0]
        pose_msg.pose.position.y = tvecs[i][0][1]
        pose_msg.pose.position.z = tvecs[i][0][2]

        # Publish the pose message
        pub.publish(pose_msg)

# Callback function for image and camera info
def image_callback(image_msg):
    print(f'status of camera_info_received: {camera_info_received}')
    return
    if not camera_info_received:
        rospy.logwarn('Camera matrix or distortion coefficients not received')
        return
    try:
        # Convert image message to OpenCV image
        image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        image_received = True
                #  Detect Aruco markers
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detectMarkers(gray, aruco_dict, parameters=parameters)
        aruco.drawDetectedMarkers(image, corners, ids)
        cv2.imshow('aruco_detect', image)
        cv2.waitKey(1)
        
        # if ids is not None:
        #     # Estimate pose of each marker
        #     rvecs, tvecs, _ = estimatePoseSingleMarkers(corners, 0.05, camera_info_msg.K, camera_info_msg.D)

        #     # Publish marker poses and timestamps
        #     publish_marker_pose(ids, rvecs, tvecs, np.array(camera_info_msg.K), np.array(camera_info_msg.D), image_msg.header.stamp)

    except CvBridgeError as e:
        rospy.logerr(f'CvBridgeError: {e}')
        return

def camera_info_callback(camera_info_msg):
    print('entered camera_info_callback')
    try:
        # Store camera matrix and distortion coefficients
        camera_matrix = np.array(camera_info_msg.K).reshape(3, 3)
        dist_coeffs = np.array(camera_info_msg.D)
        camera_info_received = True
        # print(f'status of camera_info_received: {camera_info_received}')
        # print(f'camera_matrix: {camera_matrix}')
        # print(f'dist_coeffs: {dist_coeffs}')
    except Exception as e:
        rospy.logerr(f'Error in camera_info_callback: {e}')
        camera_info_received = False

if __name__ == '__main__':
    # ROS node initialization
    rospy.init_node('aruco_marker_detector')

    # Image and camera info subscribers
    camera_info_sub = rospy.Subscriber('/sony_cam2/camera_info', CameraInfo, camera_info_callback)
    rospy.sleep(1)
    image_sub = rospy.Subscriber('/sony_cam2/image_raw', Image, image_callback)


    # Bridge for converting image messages
    bridge = CvBridge()

    rospy.spin()
