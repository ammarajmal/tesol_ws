#include "CameraApi.h" // Camera SDK API header

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>

using namespace cv;

unsigned char *g_pRgbBuffer; // Buffer to store processed image data (converted to RGB for OpenCV)

int main()
{
    int iCameraCounts = 1;                    // Number of cameras detected
    int iStatus = -1;                         // Status code for SDK function calls
    tSdkCameraDevInfo tCameraEnumList;        // Array storing discovered camera device info
    int hCamera;                              // Handle/reference to the initialized camera
    tSdkCameraCapbility tCapability;          // Structure that describes the capabilities of the connected camera
    tSdkFrameHead sFrameInfo;                 // Metadata about each frame (resolution, format, etc.)
    BYTE *pbyBuffer;                          // Raw image buffer from SDK
    int iDisplayFrames = 10000;               // Number of frames to display (loop exit condition)
    int channel = 3;                          // Image channel count (1 = mono, 3 = color)

    // Initialize the camera SDK. Must be called before any other SDK function.
    CameraSdkInit(1);

    // Enumerate connected devices and populate the device list
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    printf("state = %d\n", iStatus);
    printf("count = %d\n", iCameraCounts);

    // Exit if no camera is found
    if (iCameraCounts == 0) {
        return -1;
    }

    // Initialize the selected camera.
    // Parameters: (device list, emulation mode -1, option -1, output camera handle)
    iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
    printf("state = %d\n", iStatus);
    if (iStatus != CAMERA_STATUS_SUCCESS) {
        return -1; // Initialization failed
    }

    // Retrieve the camera capability structure,
    // which contains information about supported resolutions, formats, features, etc.
    CameraGetCapability(hCamera, &tCapability);

    // Allocate memory for the RGB image buffer.
    // Buffer size = max width × max height × number of channels
    g_pRgbBuffer = (unsigned char*)malloc(
        tCapability.sResolutionRange.iHeightMax * 
        tCapability.sResolutionRange.iWidthMax * 
        3
    );
    // Alternative or legacy allocation method (commented out)
    // g_readBuf = (unsigned char*)malloc(...);

    /*
    Start the camera acquisition engine.
    This prepares the SDK to receive streaming image data from the camera.
    - For trigger mode cameras, images are only received when a trigger is detected.
    - For continuous mode, frames are streamed automatically.
    */
    CameraPlay(hCamera);

    /*
    Additional camera configuration examples (not fully demonstrated here but possible):
    - Exposure time: CameraSetExposureTime / CameraGetExposureTime
    - Resolution: CameraSetImageResolution / CameraGetImageResolution
    - Gain, Gamma, Contrast: CameraSetGain, CameraSetGamma, CameraSetContrast, etc.
    This example focuses only on image acquisition and conversion to OpenCV format.
    */

    // Set output pixel format based on sensor type (Mono or Color)
    if (tCapability.sIspCapacity.bMonoSensor) {
        channel = 1;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
    } else {
        channel = 3;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8); // Format suitable for OpenCV
    }

    // Display loop: capture and display a fixed number of frames
    while (iDisplayFrames--) {
        // Try to capture a frame within a 1000 ms timeout
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
            
            // Convert raw image (sensor format) to RGB/BGR format
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

            // Convert image buffer into OpenCV Mat for processing/display
            cv::Mat matImage(
                cv::Size(sFrameInfo.iWidth, sFrameInfo.iHeight),
                sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                g_pRgbBuffer
            );

            // Show image using OpenCV GUI window
            imshow("Opencv Demo", matImage);
            waitKey(5); // Delay and GUI refresh

            /*
            IMPORTANT:
            After successfully calling CameraGetImageBuffer,
            you MUST call CameraReleaseImageBuffer to release the internal buffer lock.
            If not released, further calls to CameraGetImageBuffer will block indefinitely
            until another thread releases the previous buffer.
            */
            CameraReleaseImageBuffer(hCamera, pbyBuffer);
        }
    }

    // Clean up: stop image acquisition and release hardware resources
    CameraUnInit(hCamera);

    // After uninitializing the camera, it's safe to free allocated memory
    free(g_pRgbBuffer);

    return 0;
}
