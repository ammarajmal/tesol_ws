#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def main():
    rospy.init_node('camera_publisher', anonymous=True)
    image_pub = rospy.Publisher('/sony_cam1/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    cap = cv2.VideoCapture('/dev/video_cam1')

    if not cap.isOpened():
        rospy.logerr("Cannot open camera")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Can't receive frame (stream end?). Exiting ...")
            break

        # Publish the frame
        image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        image_pub.publish(image_msg)

        # Optionally display the frame
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) == ord('q'):
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
