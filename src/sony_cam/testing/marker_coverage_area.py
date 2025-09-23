#!/usr/bin/env python3
"""
ROS node to detect Apriltag 36h11 in a camera feed and calculate pixel coverage percentage
Usage: rosrun <package> apriltag_coverage.py /camera_topic_name
"""

import rospy
import sys
import numpy as np
import cv2
import apriltag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class AprilTagCoverage:
    def __init__(self, camera_topic):
        self.camera_topic = camera_topic
        self.bridge = CvBridge()
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))
        rospy.init_node('apriltag_coverage', anonymous=True)
        rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        rospy.loginfo(f"Subscribed to {self.camera_topic}")

    def image_callback(self, data):
        # Convert ROS Image to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='mono8')  # Apriltag works fine on grayscale
        
        # Detect apriltags
        detections = self.detector.detect(cv_image)
        
        height, width = cv_image.shape
        total_pixels = height * width
        
        if not detections:
            rospy.loginfo("No Apriltag detected in current frame.")
            return
        
        for det in detections:
            # The detection polygon gives corners pixel coordinates of the apriltag
            corners = det.corners
            points = np.array(corners, dtype=np.int32)
            
            # Create a mask for the apriltag polygon
            mask = np.zeros_like(cv_image, dtype=np.uint8)
            cv2.fillPoly(mask, [points], 255)
            
            # Count pixels inside the apriltag polygon
            apriltag_pixels = cv2.countNonZero(mask)
            
            # Compute coverage percentage
            coverage_percent = (apriltag_pixels / total_pixels) * 100
            
            rospy.loginfo(f"Apriltag detected with pixel coverage: {coverage_percent:.2f}%")
            print(f"Apriltag pixel area: {apriltag_pixels}, Total image pixels: {total_pixels}, Coverage: {coverage_percent:.2f}%")
            # We report for first tag only (if multiple, could iterate or choose max)
            break


def main():
    if len(sys.argv) < 2:
        print("Usage: rosrun <package> apriltag_coverage.py /camera_topic_name")
        sys.exit(1)

    camera_topic = sys.argv[1]
    apriltag_coverage = AprilTagCoverage(camera_topic)
    rospy.spin()


if __name__ == '__main__':
    main()
