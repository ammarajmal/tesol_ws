#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import rospkg
from camera_info_manager import CameraInfoManager
import yaml

def read_png_image(file_path):
    try:
        image = cv2.imread(file_path, cv2.IMREAD_COLOR)
        if image is None:
            raise Exception(f"Could not read image from {file_path}")
        return image
    except Exception as e:
        rospy.logerr(f"Error reading PNG image: {e}")
        return None

class FakeCameraNode:
    def __init__(self):
        rospy.init_node('fake_camera_node', anonymous=False)

        self.camera_name = rospy.get_param('~camera_name', 'sony_cam1')
        self.image_width = rospy.get_param('~image_width', 640)
        self.image_height = rospy.get_param('~image_height', 480)
        self.frame_rate = rospy.get_param('~frame_rate', 60)
        self.camera_frame_id = rospy.get_param('~camera_frame_id', 'sony_cam1')
        self.camera_info_url = rospy.get_param('~camera_info_url', '')
        self.png_file_path = rospy.get_param('~png_file_path', 'image.png')

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('~image_raw', Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher('~camera_info', CameraInfo, queue_size=10)
        self.image_sub = rospy.Subscriber('~image_raw', Image, self.image_callback)
        self.srv_set_camera_info = rospy.Service('~set_camera_info', SetCameraInfo, self.set_camera_info_callback)

        rospy.loginfo(f"Initializing CameraInfoManager with URL: {self.camera_info_url}")
        self.cam_info_manager = CameraInfoManager(cname=self.camera_name, url=self.camera_info_url)

        self.check_calibration_file()

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('sony_cam')
        self.image_path = os.path.join(package_path, 'node/image', self.png_file_path)
        rospy.loginfo(f"Loading PNG image from path: {self.image_path}")
        self.image = read_png_image(self.image_path)
        if self.image is not None:
            rospy.loginfo("PNG image loaded successfully.")
        else:
            rospy.logerr("Failed to load PNG image.")

        self.loop_rate = rospy.Rate(self.frame_rate)
    
    def check_calibration_file(self):
        calibration_file_path = self.camera_info_url.replace("file://", "")
        if os.path.isfile(calibration_file_path):
            rospy.loginfo(f"Camera calibration file found at: {calibration_file_path}")
            try:
                with open(calibration_file_path, 'r') as f:
                    content = f.read()
                    rospy.loginfo(f"Camera calibration file content:\n{content}")
                try:
                    calib = yaml.load(content, Loader=yaml.SafeLoader)
                    rospy.loginfo(f"Parsed calibration data: {calib}")
                except yaml.YAMLError as e:
                    rospy.logerr(f"Error parsing calibration file: {e}")
                rospy.loginfo("Loading CameraInfoManager with parsed calibration data")
                if not self.cam_info_manager.loadCameraInfo():
                    rospy.logwarn("Camera calibration file not found or invalid, continuing without calibration.")
                    print(self.cam_info_manager.getCameraInfo())

                elif not self.cam_info_manager.isCalibrated():
                    rospy.logwarn("Camera is not calibrated!")
                else:
                    rospy.loginfo("Camera calibration loaded successfully.")
            except Exception as e:
                rospy.logerr(f"Error reading calibration file: {e}")
        else:
            rospy.logerr(f"Camera calibration file not found at: {calibration_file_path}")

    def set_camera_info_callback(self, req):
        success = self.cam_info_manager.setCameraInfo(req.camera_info)
        res = SetCameraInfoResponse()
        res.success = success
        if success:
            res.status_message = "Camera info set successfully."
        else:
            res.status_message = "Failed to set camera info."
        return res

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Output Image", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")

    def publish_image(self):
        while not rospy.is_shutdown():
            if self.image is not None:
                ros_image = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
                ros_image.header.frame_id = self.camera_frame_id
                ros_image.header.stamp = rospy.Time.now()
                self.image_pub.publish(ros_image)

                camera_info = self.cam_info_manager.getCameraInfo()
                camera_info.header.frame_id = self.camera_frame_id
                camera_info.header.stamp = ros_image.header.stamp
                self.camera_info_pub.publish(camera_info)

            self.loop_rate.sleep()

if __name__ == '__main__':
    try:
        node = FakeCameraNode()
        node.publish_image()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
