#!/usr/bin/env python3
"""
===============================================================================
 Project      : Multi-Camera Image Acquisition Demo
 File         : MultiCameraAcquisition.py
 Description  : Simultaneously captures and displays live video from three cameras,
                each with real-time FPS overlay in its display window.
                Uses MindVision SDK (mvsdk) and OpenCV.
 Author       : Ammar Ajmal
 Email        : ammarajml@gmail.com
 Created      : 2025-07-27
 Last Edited  : 2025-07-27
 Version      : 1.0
 Location     : Seoul, South Korea
===============================================================================
"""

import threading
import queue
import time
import mvsdk
import cv2
import numpy as np

def camera_worker(hCamera, width, height, channels, frame_queue, cam_name):
    buf_size = width * height * channels
    pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)
    frame_count = 0
    fps = 0
    t_start = time.time()
    mono = (channels == 1)
    try:
        while True:
            try:
                pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 1000)
                mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
                mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
            except mvsdk.CameraException:
                continue
            frame_bytes = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_bytes, dtype=np.uint8)
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, channels))
            frame_count += 1
            elapsed = time.time() - t_start
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                t_start = time.time()
            color = (0,255,0) if not mono else (255,)
            cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
            # Only put latest frame in queue; clear old if needed
            if frame_queue.full():
                try: frame_queue.get_nowait() 
                except queue.Empty: pass
            frame_queue.put(frame)
    finally:
        mvsdk.CameraAlignFree(pFrameBuffer)
        mvsdk.CameraUnInit(hCamera)

def main():
    import sys
    dev_list = mvsdk.CameraEnumerateDevice()
    n = min(3, len(dev_list))
    if n < 1:
        print("No cameras found.")
        return

    threads = []
    frame_queues = [queue.Queue(maxsize=1) for _ in range(n)]
    win_names = []

    for idx, dev in enumerate(dev_list[:n]):
        hCamera = mvsdk.CameraInit(dev, -1, -1)
        cap = mvsdk.CameraGetCapability(hCamera)
        mono = bool(cap.sIspCapacity.bMonoSensor)
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8 if mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8)
        channels = 1 if mono else 3
        width = cap.sResolutionRange.iWidthMax
        height = cap.sResolutionRange.iHeightMax
        mvsdk.CameraSetTriggerMode(hCamera, 0)
        mvsdk.CameraSetAeState(hCamera, 0)
        mvsdk.CameraSetExposureTime(hCamera, 30*1000)
        mvsdk.CameraSetFrameSpeed(hCamera, 3)
        mvsdk.CameraPlay(hCamera)
        cam_name = f"Cam{idx+1} View"
        win_names.append(cam_name)
        t = threading.Thread(target=camera_worker,
                             args=(hCamera, width, height, channels, frame_queues[idx], cam_name),
                             daemon=True)
        t.start()
        threads.append(t)

    try:
        while True:
            for idx, q in enumerate(frame_queues):
                if not q.empty():
                    frame = q.get()
                    cv2.imshow(win_names[idx], frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        for w in win_names:
            cv2.destroyWindow(w)

if __name__ == "__main__":
    main()
        