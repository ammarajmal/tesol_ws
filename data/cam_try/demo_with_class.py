#coding=utf-8
import cv2
import numpy as np
import mvsdk
import time
import platform

class App(object):
    def init(self):
        super(App, self).init()
        self.pFrameBuffer = 0
        self.quit = False
    def main(self):
        # Enumerate the camera devices
        DevList = mvsdk.CameraEnumerateDevice()
        nDev = len(DevList)
        if nDev < 1:
            print("No camera was found!")
            return

        for i, DevInfo in enumerate(DevList):
            print(f"{i}: {DevInfo.GetFriendlyName()} {DevInfo.GetPortType()}")
        i = 0 if nDev == 1 else int(input("Select camera: "))
        DevInfo = DevList[i]
        print(DevInfo)

        # Open the camera
        hCamera = 0
        try:
            hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
        except mvsdk.CameraException as e:
            print(f"CameraInit Failed({e.error_code}): {e.message}")
            return

        # Get the camera features description
        cap = mvsdk.CameraGetCapability(hCamera)

        # Determine if the camera is a monochrome camera or a color camera
        monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

        # For monochrome camera, make the ISP output MONO data directly, instead of extending to 24-bit gray of R=G=B.
        if monoCamera:
            mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
            mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

        # Switch the camera mode to continuous acquisition
        mvsdk.CameraSetTriggerMode(hCamera, 0)

        # Manually set exposure time to 30ms
        mvsdk.CameraSetAeState(hCamera, 0)
        mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)

        # Start the internal image acquisition thread of the SDK
        mvsdk.CameraPlay(hCamera)

        # Calculate the size of the RGB buffer needed, here directly allocate according to the maximum resolution of the camera.
        FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)

        # Allocate RGB buffer to store the image output by ISP
        # Note: The RAW data is transmitted from the camera to the PC. On the PC side, the software ISP is used to convert it to RGB data 
        # (if it is a monochrome camera, there is no need to convert the format, but ISP has other processing, so this buffer also needs to be allocated)
        self.pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

        # Set the acquisition callback function
        self.quit = False
        mvsdk.CameraSetCallbackFunction(hCamera, self.GrabCallback, 0)

        # Wait for exit
        while not self.quit:
            time.sleep(0.1)

        # Close the camera
        mvsdk.CameraUnInit(hCamera)

        # Release the frame buffer
        mvsdk.CameraAlignFree(self.pFrameBuffer)

    @mvsdk.method(mvsdk.CAMERA_SNAP_PROC)
    def GrabCallback(self, hCamera, pRawData, pFrameHead, pContext):
        FrameHead = pFrameHead[0]
        pFrameBuffer = self.pFrameBuffer

        mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

		# For Windows, the image data is upside down and stored in BMP format.
		# To convert it to OpenCV format, it needs to be flipped vertically
		# On Linux, the image is already in the correct orientation
        if platform.system() == "Windows":
            mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)

		# The image is now stored in pFrameBuffer.
		# For a color camera, pFrameBuffer contains RGB data; for a monochrome camera, it contains 8-bit grayscale data.
		# Convert the pFrameBuffer to the OpenCV image format for subsequent algorithm processing
        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3) )

		# Resize the image to (640, 480) for display
        frame = cv2.resize(frame, (640,480), interpolation = cv2.INTER_LINEAR)
        cv2.imshow("Press q to end", frame)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            self.quit = True

def main():
	try:
		app = App()
		app.main()
	finally:
		cv2.destroyAllWindows()

main()