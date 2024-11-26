#!/usr/bin/env python3
"""
file to subscribe the camera /sony_cam3/image_raw and publish the resolution of the image"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback(data):
    bridge = CvBridge()
    print("Received an image!")
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    height, width, channels = cv_image.shape
    rospy.loginfo("height: %s, width: %s", height, width)
    
def resolution():
    rospy.init_node('resolution', anonymous=True)
    rospy.Subscriber('/sony_cam2/image_raw', Image, callback)
    rospy.spin()
    
if __name__ == '__main__':
    resolution()
    