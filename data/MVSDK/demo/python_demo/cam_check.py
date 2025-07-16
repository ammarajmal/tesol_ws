import mvsdk
import cv2
import numpy as np
import time

# Map serial numbers to user-friendly names
CAMERA_SN_MAP = {
    "053012620218": "CAM1",
    "052120120268": "CAM2",
    "052120120267": "CAM3"
}

# Media type string representations for debug printing
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
    dev_list = mvsdk.CameraEnumerateDevice()
    if not dev_list:
        print("No cameras found.")
        return

    # Display available cameras
    sn_to_dev = {}
    print("Detected Cameras:")
    for dev in dev_list:
        sn = dev.GetSn()
        fname = dev.GetFriendlyName()
        logical_name = CAMERA_SN_MAP.get(sn, "UNKNOWN")
        print(f"  {logical_name}: Model={fname}, SN={sn}")
        if logical_name != "UNKNOWN":
            sn_to_dev[logical_name] = dev

    options = list(sn_to_dev.keys())
    print("\nAvailable:", ', '.join(options))
    cam_choice = input("Enter camera to connect (CAM1 / CAM2 / CAM3): ").strip().upper()
    if cam_choice not in sn_to_dev:
        print(f"Invalid camera: {cam_choice}")
        return

    dev = sn_to_dev[cam_choice]

    try:
        hCamera = mvsdk.CameraInit(dev, -1, -1)
    except mvsdk.CameraException as e:
        print(f"CameraInit failed: {e}")
        return

    cap = mvsdk.CameraGetCapability(hCamera)
    mono = bool(cap.sIspCapacity.bMonoSensor)

    # Set correct output format
    if mono:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        channels = 1
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)
        channels = 3

    # Configure camera settings
    mvsdk.CameraSetTriggerMode(hCamera, 0)
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 10000)
    mvsdk.CameraSetFrameSpeed(hCamera, 3)  # Super speed mode
    mvsdk.CameraPlay(hCamera)

    # Allocate frame buffer
    width = cap.sResolutionRange.iWidthMax
    height = cap.sResolutionRange.iHeightMax
    buf_size = width * height * channels
    pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)

    print(f"\n{cam_choice} streaming started. Press 'q' to quit.")

    # FPS tracking
    t_start = time.time()
    frame_count = 0
    fps = 0
    printed_media_type = False

    try:
        while True:
            try:
                pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 1000)
                mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
                mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
            except mvsdk.CameraException as e:
                if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                    print(f"Image buffer error: {e}")
                continue

            # Print media type info once
            if not printed_media_type:
                media_type = FrameHead.uiMediaType
                media_str = MEDIA_TYPE_STR.get(media_type, f"Unknown (0x{media_type:X})")
                print(f"Frame Media Type: {media_str}")
                printed_media_type = True

            # Convert frame buffer to numpy image
            frame_bytes = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_bytes, dtype=np.uint8)
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if mono else 3))
            frame = cv2.resize(frame, (640, 480))

            # FPS calculation
            frame_count += 1
            elapsed = time.time() - t_start
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                t_start = time.time()

            # Overlay FPS on image
            cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

            # Show image
            cv2.imshow(f"{cam_choice} View", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        mvsdk.CameraUnInit(hCamera)
        mvsdk.CameraAlignFree(pFrameBuffer)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
