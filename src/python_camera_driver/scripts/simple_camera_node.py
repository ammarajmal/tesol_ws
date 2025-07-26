#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import mvsdk
import cv2
import numpy as np

def main():
    # 1. ROS Setup
    rospy.init_node('simple_camera_node')
    
    # Get parameters from the launch file
    # Default to CAM1 if not specified
    target_cam_id = rospy.get_param('~camera_id', 'CAM1') 
    # Get the serial number for the desired camera ID
    cam_sn_map = rospy.get_param('~camera_sn_map', {})
    
    # Invert the map to find SN from ID
    sn_to_id = {v: k for k, v in cam_sn_map.items()}
    target_sn = sn_to_id.get(target_cam_id)

    if not target_sn:
        rospy.logerr(f"Camera ID '{target_cam_id}' not found in serial number map.")
        return

    # ROS Publishers and tools
    image_pub = rospy.Publisher('image_raw', Image, queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(300) # 30 Hz

    # 2. Camera SDK Initialization
    dev_list = mvsdk.CameraEnumerateDevice()
    if not dev_list:
        rospy.logerr("No cameras found.")
        return

    # Find the device info for our target camera
    target_dev_info = None
    for dev in dev_list:
        if dev.GetSn() == target_sn:
            target_dev_info = dev
            break
    
    if not target_dev_info:
        rospy.logerr(f"Camera with S/N {target_sn} ({target_cam_id}) not connected.")
        return

    try:
        hCamera = mvsdk.CameraInit(target_dev_info, -1, -1)
    except mvsdk.CameraException as e:
        rospy.logerr(f"CameraInit failed for {target_cam_id}: {e}")
        return

    # 3. Configure Camera
    cap = mvsdk.CameraGetCapability(hCamera)
    is_mono = cap.sIspCapacity.bMonoSensor != 0
    encoding = "mono8" if is_mono else "bgr8"
    channels = 1 if is_mono else 3

    if is_mono:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

    mvsdk.CameraSetTriggerMode(hCamera, 0) # Continuous mode
    mvsdk.CameraSetAeState(hCamera, 1)     # Enable auto exposure
    mvsdk.CameraSetExposureTime(hCamera, 100000) # 100ms exposure
    mvsdk.CameraPlay(hCamera)

    # Allocate buffer
    buf_size = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * channels
    pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)
    rospy.loginfo(f"Streaming started for {target_cam_id} (S/N: {target_sn})")

    # 4. Main Loop
    while not rospy.is_shutdown():
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

            # Convert buffer to NumPy image
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, channels))

            # Publish the image
            ros_image_msg = bridge.cv2_to_imgmsg(frame, encoding=encoding)
            ros_image_msg.header.stamp = rospy.Time.now()
            ros_image_msg.header.frame_id = target_cam_id
            image_pub.publish(ros_image_msg)

        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                rospy.logwarn(f"Image buffer error: {e}")
            continue
        
        rate.sleep()

    # 5. Cleanup
    rospy.loginfo(f"Shutting down camera {target_cam_id}.")
    mvsdk.CameraUnInit(hCamera)
    mvsdk.CameraAlignFree(pFrameBuffer)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass