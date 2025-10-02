#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time

class TagPixelCoverage:
    def __init__(self):
        rospy.init_node("tag_pixel_coverage", anonymous=True)

        # Subscribe to sony_cam2 feed
        self.image_sub = rospy.Subscriber("/sony_cam2/image_raw", Image, self.callback)
        self.bridge = CvBridge()

        # AprilTag dictionary (36h11 -> corresponds to aruco.DICT_APRILTAG_36h11)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        self.parameters = aruco.DetectorParameters_create()

        self.last_time = time.time()
        self.fps = 0

    def callback(self, msg):
        # Convert ROS Image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            for c in corners:
                # Draw bounding box
                int_c = np.int32(c)
                cv2.polylines(frame, int_c, True, (0, 255, 0), 2)

                # Compute pixel coverage (marker width vs image width)
                x_min = int(np.min(c[0][:, 0]))
                x_max = int(np.max(c[0][:, 0]))
                tag_width_px = x_max - x_min
                img_width = frame.shape[1]

                coverage = (tag_width_px / img_width) * 100

                # Annotate coverage
                cv2.putText(frame, f"Coverage: {coverage:.2f}%", (x_min, int(np.min(c[0][:, 1])) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # FPS calculation
        now = time.time()
        self.fps = 1.0 / (now - self.last_time)
        self.last_time = now

        cv2.putText(frame, f"FPS: {self.fps:.1f}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        # Show output
        cv2.imshow("sony_cam2 - AprilTag Coverage", frame)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    node = TagPixelCoverage()
    node.run()
