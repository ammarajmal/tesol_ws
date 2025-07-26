#!/usr/bin/env python3
"""
===============================================================================
 Project      : Camera Image Acquisition Demo
 File         : CameraAcquisition.py
 Description  : Captures and displays live camera frames with real-time FPS.
                Uses MindVision SDK (`mvsdk`) and OpenCV. 
                Efficient, robust, easy to use, and beginner-friendly.
 Author       : Ammar Ajmal
 Email        : ammarajml@gmail.com
 Created      : 2025-07-26
 Last Edited  : 2025-07-26
 Version      : 1.1
 Location     : Seoul, South Korea
 Dependencies : MindVision SDK (mvsdk), OpenCV (cv2), NumPy
 
 Usage:
     - Connect your supported camera(s).
     - Run this program.
     - Select the camera by index (1, 2, or 3).
     - Press 'q' in the display window to exit.
 
 Change Log:
     - 2025-07-26 v1.0: Initial code for image acquisition and FPS overlay.
     - 2025-07-26 v1.1: User selects camera by number, displays full resolution.
 
 License: (Add license notice here, e.g., MIT, GPL, proprietary)
===============================================================================
"""

import mvsdk
import cv2
import numpy as np
import time
import sys

# Optional user-friendly camera naming by SN
CAMERA_SN_MAP = {
    "053012620218": "CAM1",
    "052120120268": "CAM2",
    "052120120267": "CAM3"
}

MEDIA_TYPE_STR = {
    mvsdk.CAMERA_MEDIA_TYPE_MONO8: "MONO8",
    mvsdk.CAMERA_MEDIA_TYPE_BGR8: "BGR8",
    mvsdk.CAMERA_MEDIA_TYPE_RGB8: "RGB8",
    mvsdk.CAMERA_MEDIA_TYPE_BAYGR8: "BAYER_GR8",
    mvsdk.CAMERA_MEDIA_TYPE_BAYRG8: "BAYER_RG8",
    mvsdk.CAMERA_MEDIA_TYPE_BAYGB8: "BAYER_GB8",
    mvsdk.CAMERA_MEDIA_TYPE_BAYBG8: "BAYER_BG8",
}


def main():
    # Enumerate connected cameras
    dev_list = mvsdk.CameraEnumerateDevice()
    if not dev_list:
        print("No cameras found.")
        return

    # Prepare camera info and mapping for numeric selection
    sn_to_dev = []
    print("Detected Cameras:")
    for idx, dev in enumerate(dev_list, 1):
        sn = dev.GetSn()
        fname = dev.GetFriendlyName()
        logical_name = CAMERA_SN_MAP.get(sn, "UNKNOWN")
        print(f"  [{idx}] Model={fname}, SN={sn}, Alias={logical_name}")
        sn_to_dev.append((sn, dev))

    # User selects camera by number
    print("\nAvailable cameras:", ', '.join(f"{i+1}" for i in range(len(sn_to_dev))))
    try:
        cam_choice = int(input("Enter camera number to connect (1/2/3): ").strip())
    except Exception:
        print("Invalid input. Exiting.")
        return
    if not (1 <= cam_choice <= len(sn_to_dev)):
        print(f"Invalid camera index: {cam_choice}")
        return

    selected_sn, dev = sn_to_dev[cam_choice - 1]
    cam_display_name = CAMERA_SN_MAP.get(selected_sn, f"CAM{cam_choice}")

    try:
        hCamera = mvsdk.CameraInit(dev, -1, -1)
    except mvsdk.CameraException as e:
        print(f"CameraInit failed: {e}")
        return

    try:
        cap = mvsdk.CameraGetCapability(hCamera)
        mono = bool(cap.sIspCapacity.bMonoSensor)

        # Set output format for the selected camera
        mvsdk.CameraSetIspOutFormat(
            hCamera,
            mvsdk.CAMERA_MEDIA_TYPE_MONO8 if mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8
        )
        channels = 1 if mono else 3

        # Camera basic configuration for optimal/fast operation
        mvsdk.CameraSetTriggerMode(hCamera, 0)        # Continuous mode
        mvsdk.CameraSetAeState(hCamera, 0)            # 0 = Manual exposure, 1 = Auto exposure
        mvsdk.CameraSetExposureTime(hCamera, 9000)   # 10 ms exposure
        mvsdk.CameraSetFrameSpeed(hCamera, 3)         # Super speed
        mvsdk.CameraPlay(hCamera)

        # Allocate SDK-aligned buffer for high efficiency (matches C++ logic)
        width = cap.sResolutionRange.iWidthMax
        height = cap.sResolutionRange.iHeightMax
        buf_size = width * height * channels
        pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)

        print(f"\n{cam_display_name} streaming started at full resolution. Press 'q' to quit.\n")

        # FPS tracking
        t_start = time.time()
        frame_count = 0
        fps = 0
        printed_media_type = False

        while True:
            try:
                pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 1000)
                mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
                mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
            except mvsdk.CameraException as e:
                if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                    print(f"Image buffer error: {e}")
                continue

            # Print media type only once for debugging
            if not printed_media_type:
                media_type = FrameHead.uiMediaType
                media_str = MEDIA_TYPE_STR.get(media_type, f"Unknown (0x{media_type:X})")
                print(f"Frame Media Type: {media_str}")
                printed_media_type = True

            # Efficient buffer to NumPy array of correct shape—NO RESIZING!
            frame_bytes = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_bytes, dtype=np.uint8)
            # Shape as (height, width) for mono, (height, width, 3) for color
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, channels))

            # FPS calculation and overlay (update every second)
            frame_count += 1
            elapsed = time.time() - t_start
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                t_start = time.time()

            fps_color = (0, 255, 0) if not mono else (255,)
            cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, fps_color, 2)

            # Show image at its actual resolution
            cv2.imshow(f"{cam_display_name} View", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        mvsdk.CameraUnInit(hCamera)
        if 'pFrameBuffer' in locals():
            mvsdk.CameraAlignFree(pFrameBuffer)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
