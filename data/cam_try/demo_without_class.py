#coding=utf-8
import cv2
import numpy as np
import mvsdk
import platform

def main_loop():
    print("main_loop")
    # Enumerate cameras
    DevList = mvsdk.CameraEnumerateDevice()
    nDev = len(DevList)
    if nDev < 1:
        print("No camera was found!")
        return
    for i, DevInfo in enumerate(DevList):
	    print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
    i = 0 if nDev == 1 else int(input("Select camera: "))
    DevInfo = DevList[i]
    print(DevInfo)

    # Open the camera
    hCamera = 0
    try:
        hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
    except mvsdk.CameraException as e:
        print("CameraInit Failed({}): {}".format(e.error_code, e.message) )
        return

    # Get camera capability description
    cap = mvsdk.CameraGetCapability(hCamera)

    # Determine whether it is a black and white camera or a color camera
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

    # For black and white cameras, let the ISP directly output MONO data instead of expanding to R=G=B 24-bit grayscale
    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

    # Switch camera mode to continuous acquisition
    mvsdk.CameraSetTriggerMode(hCamera, 0)

    # Manual exposure, exposure time 30ms
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)

    # Start the SDK internal image acquisition thread
    mvsdk.CameraPlay(hCamera)

    # Calculate the size of the RGB buffer required, here we directly allocate according to the maximum resolution of the camera
    FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)

    # Allocate the RGB buffer to store the output image of the ISP
    # Note: The RAW data is transmitted from the camera to the PC side, and the RGB data is converted by the software ISP on the PC side (if it is a black and white camera, no format conversion is required, but the ISP has other processing, so this buffer is also required)
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

    while (cv2.waitKey(1) & 0xFF) != ord('q'):
        # Get one frame from the camera
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

            # The image data obtained in Windows is upside down, stored in BMP format. To convert to OpenCV, you need to flip it up and down
            # On Linux, the output is correct and does not need to be flipped up and down
            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)
            # At this point, the image is stored in pFrameBuffer. For color cameras, pFrameBuffer=RGB data, and for black and white cameras, pFrameBuffer=8 
            # bit grayscale data
            # Convert pFrameBuffer to OpenCV image format for subsequent algorithm processing
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))
            frame = cv2.resize(frame, (640,480), interpolation = cv2.INTER_LINEAR)
            cv2.imshow("Press q to end", frame)
        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message) )

    # Close the camera
    mvsdk.CameraUnInit(hCamera)

    # Free frame buffer
    mvsdk.CameraAlignFree(pFrameBuffer)
    
def main():
    try:
        print('into the main')
        main_loop()
    finally:
        cv2.destroyAllWindows()
main()
