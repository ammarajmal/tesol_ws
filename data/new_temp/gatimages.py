#!/usr/env/bin python3
#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

# Initialize the CvBridge to convert ROS Image messages to OpenCV format
bridge = CvBridge()

# Directory to save the images
save_directory = './captured_images/'
if not os.path.exists(save_directory):
    os.makedirs(save_directory)

# Initialize counter for saved images
image_count = 0
max_images = 30

def image_callback(msg):
    global image_count

    # Convert ROS Image message to OpenCV image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    # Save the image
    image_filename = os.path.join(save_directory, f'image_{image_count + 1}.png')
    cv2.imwrite(image_filename, cv_image)
    rospy.loginfo(f"Saved image {image_count + 1} to {image_filename}")

    # Increment the counter
    image_count += 1

    # Stop after 30 images
    if image_count >= max_images:
        rospy.signal_shutdown("Captured 30 images, shutting down.")

def capture_images():
    rospy.init_node('image_capture_node', anonymous=True)

    # Subscribe to the camera topic
    rospy.Subscriber('/sony_cam3/image_raw', Image, image_callback)

    # Keep the node running until 30 images are captured
    rospy.loginfo("Waiting for images...")
    rospy.spin()

if __name__ == '__main__':
    try:
        capture_images()
    except rospy.ROSInterruptException:
        pass
