#!/usr/bin/env python3
"""
Subscribe to a camera image topic and publish the resolution of the received images.

Usage:
    rosrun <package_name> <script_name>.py /camera_topic_name
Example:
    rosrun <package_name> resolution.py /sony_cam2/image_raw
"""

import rospy
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def callback(data):
    bridge = CvBridge()
    print("Received an image!")
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    height, width, channels = cv_image.shape
    rospy.loginfo("height: %s, width: %s", height, width)


def resolution(camera_topic):
    rospy.init_node('resolution', anonymous=True)
    rospy.Subscriber(camera_topic, Image, callback)
    rospy.loginfo("Subscribed to %s", camera_topic)
    rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: rosrun <package_name> <script_name>.py /camera_topic_name")
        sys.exit(1)
    camera_topic = sys.argv[1]
    resolution(camera_topic)
