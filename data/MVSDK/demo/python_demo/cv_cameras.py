#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import mvsdk
import time
import platform

class App(object):
	def __init__(self):
		super(App, self).__init__()
		self.pFrameBuffer = 0
		self.quit = False

	def main(self):
		# Enumerate all connected cameras
		DevList = mvsdk.CameraEnumerateDevice()
		nDev = len(DevList)

		if nDev < 1:
			print("No camera was found!")
			return

		# Print detected camera info
		for i, DevInfo in enumerate(DevList):
			print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))

		# Let user select a camera if more than one is connected
		i = 0 if nDev == 1 else int(input("Select camera index: "))
		DevInfo = DevList[i]
		print(f"Selected camera: {DevInfo.GetFriendlyName()}")

		# Open camera and obtain a handle
		try:
			hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
		except mvsdk.CameraException as e:
			print("CameraInit Failed({}): {}".format(e.error_code, e.message))
			return

		# Get device capabilities and sensor type
		cap = mvsdk.CameraGetCapability(hCamera)
		monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

		# Set ISP output format — use MONO8 for grayscale, BGR8 for color
		if monoCamera:
			mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
		else:
			mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

		# Set camera to continuous acquisition mode (free-run)
		mvsdk.CameraSetTriggerMode(hCamera, 0)

		# Use manual exposure; set exposure to 30ms (30000 microseconds)
		mvsdk.CameraSetAeState(hCamera, 0)
		mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)

		# Start the SDK internal image acquisition and processing thread
		mvsdk.CameraPlay(hCamera)

		# Allocate buffer for processed RGB or MONO image from ISP pipeline
		FrameBufferSize = cap.sResolutionRange.iWidthMax * \
						  cap.sResolutionRange.iHeightMax * \
						  (1 if monoCamera else 3)

		self.pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

		# Set the image grabbing callback function
		self.quit = False
		mvsdk.CameraSetCallbackFunction(hCamera, self.GrabCallback, 0)

		# Main loop: runs until user exits the program
		while not self.quit:
			time.sleep(0.1)

		# Clean up: stop camera, free resources
		mvsdk.CameraUnInit(hCamera)
		mvsdk.CameraAlignFree(self.pFrameBuffer)

	@mvsdk.method(mvsdk.CAMERA_SNAP_PROC)
	def GrabCallback(self, hCamera, pRawData, pFrameHead, pContext):
		FrameHead = pFrameHead[0]
		pFrameBuffer = self.pFrameBuffer

		# Process raw frame into RGB or MONO image using ISP
		mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)

		# Release raw buffer — always required after CameraGetImageBuffer()
		mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

		# On Windows, image data may be vertically flipped — fix it if needed
		if platform.system() == "Windows":
			mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)

		# Load processed image data into a NumPy array
		frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
		frame = np.frombuffer(frame_data, dtype=np.uint8)
		frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth,
		                       1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))

		# Resize image to display window size
		frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)

		# Show image in OpenCV window
		cv2.imshow("Press q to quit", frame)

		# Exit if 'q' is pressed
		if (cv2.waitKey(1) & 0xFF) == ord('q'):
			self.quit = True

def main():
	try:
		app = App()
		app.main()
	finally:
		cv2.destroyAllWindows()

main()
