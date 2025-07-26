/******************************************************************************
 * Project      : Camera Image Acquisition Demo
 * File         : CameraAcquisition.cpp
 * Description  : Captures and displays live camera frames with real-time FPS.
 *                Uses Camera SDK and OpenCV. Highly robust, efficient, and 
 *                beginner-friendly with full documentation and error handling.
 * Author       : Ammar Ajmal
 * Email        : ammarajml@gmail.com
 * Created      : 2025-07-26
 * Last Edited  : 2025-07-26
 * Version      : 1.1
 * Location     : Seoul, South Korea
 * Compiler     : (e.g.) g++ 9.4.0 / Microsoft Visual Studio 2022
 * Dependencies : Camera SDK (CameraApi.h), OpenCV 3.x or newer
 *
 * Usage:
 *   - Connect your supported camera device(s).
 *   - Build and run this program.
 *   - Select the desired camera by index.
 *   - Press 'ESC' or 'q' in the display window to exit early.
 *
 * Change Log:
 *   - 2025-07-26 v1.0: Initial code for image acquisition and FPS overlay.
 *   - 2025-07-26 v1.1: Added user selection for multiple connected cameras.
 *
 * License: (Add a license notice here if required, e.g., MIT, GPL, proprietary)
 ******************************************************************************/

// ---- HEADER FILES ----
#include "CameraApi.h"                   // Camera SDK header (custom for your camera vendor)
#include "opencv2/core/core.hpp"         // OpenCV core functionality
#include "opencv2/highgui/highgui.hpp"   // OpenCV for GUI (imshow, waitKey, etc.)
#include "opencv2/imgproc/imgproc.hpp"   // OpenCV for image processing (putText, etc.)

#include <stdio.h>                       // Standard IO for printf
#include <iostream>                      // For C++ style error output and std::cin
#include <chrono>                        // For precise time measurement (FPS)
#include <memory>                        // Smart pointers (for resource management)

// ---- NAMESPACE ----
using namespace cv;

// ---- MAIN FUNCTION ----
int main() {
    // -- Camera device/query variables
    int iCameraCounts = 8;                    // Max number of cameras to enumerate
    int iStatus = -1;

    // Array to hold info for up to 8 cameras; adjust size as needed
    tSdkCameraDevInfo tCameraEnumList[8] = {}; 
    int hCamera = 0;                          // Camera handle
    tSdkCameraCapbility tCapability = {};    // Camera capabilities
    tSdkFrameHead sFrameInfo = {};            // Frame metadata
    BYTE* pbyBuffer = nullptr;                // Raw frame data pointer
    int channel = 3;                          // Number of image channels (color = 3, mono = 1)
    const int iDisplayFrames = 10000;         // Total frames to display (or use INT_MAX for continuous)
    int remainingFrames = iDisplayFrames;

    // -- Print OpenCV and language version info
    printf("OpenCV Version: %s\n", CV_VERSION);
#if __cplusplus == 201402L
    printf("C++ Version: C++14\n");
#elif __cplusplus == 201703L
    printf("C++ Version: C++17\n");
#else
    printf("C++ Version: Unknown\n");
#endif

    // ---- CAMERA SDK INITIALIZATION ----
    printf("Initializing Camera SDK...\n");
    iStatus = CameraSdkInit(1);
    if (iStatus != CAMERA_STATUS_SUCCESS) {
        std::cerr << "Failed to initialize Camera SDK. Error code: " << iStatus << std::endl;
        return -1;
    }

    // ---- ENUMERATE CONNECTED CAMERAS ----
    iStatus = CameraEnumerateDevice(tCameraEnumList, &iCameraCounts);
    if (iStatus != CAMERA_STATUS_SUCCESS || iCameraCounts == 0) {
        std::cerr << "No cameras found or error enumerating cameras.\n";
        return -1;
    }

    // ---- LIST FOUND CAMERAS ----
    printf("Found %d camera(s):\n", iCameraCounts);
    for (int i = 0; i < iCameraCounts; ++i) {
        printf(" [%d] Model: %s, SN: %s\n", i,
               tCameraEnumList[i].acProductName,
               tCameraEnumList[i].acSn);
    }

    // ---- PROMPT USER TO SELECT A CAMERA ----
    int selectedIndex = -1;
    std::cout << "Enter the index (0-" << iCameraCounts - 1 << ") of the camera to open: ";
    std::cin >> selectedIndex;

    // Validate input
    if (std::cin.fail() || selectedIndex < 0 || selectedIndex >= iCameraCounts) {
        std::cerr << "Invalid selection. Exiting." << std::endl;
        return -1;
    }

    // ---- INITIALIZE THE SELECTED CAMERA ----
    iStatus = CameraInit(&(tCameraEnumList[selectedIndex]), -1, -1, &hCamera);
    if (iStatus != CAMERA_STATUS_SUCCESS) {
        std::cerr << "Failed to initialize camera. Error code: " << iStatus << std::endl;
        return -1;
    }

    CameraGetCapability(hCamera, &tCapability);

    // ---- ALLOCATE IMAGE BUFFER (using smart pointer for safe memory management) ----
    std::unique_ptr<unsigned char[]> g_pRgbBuffer(
        new unsigned char[tCapability.sResolutionRange.iHeightMax *
                         tCapability.sResolutionRange.iWidthMax * 3]);

    // ---- SET CAMERA WORK MODE TO PLAY ----
    iStatus = CameraPlay(hCamera);
    if (iStatus != CAMERA_STATUS_SUCCESS) {
        std::cerr << "Failed to start camera play mode. Error code: " << iStatus << std::endl;
        CameraUnInit(hCamera);
        return -1;
    }

    // ---- SET OUTPUT IMAGE FORMAT BASED ON CAMERA SENSOR ----
    if (tCapability.sIspCapacity.bMonoSensor) {
        channel = 1;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
    } else {
        channel = 3;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
    }

    // ---- CREATE DISPLAY WINDOW ONCE ----
    namedWindow("OpenCV Camera Demo", WINDOW_AUTOSIZE);

    // ---- INITIALIZE FPS CALCULATION VARIABLES ----
    int frameCount = 0;
    double fps = 0.0;
    auto start = std::chrono::high_resolution_clock::now();

    // ---- MAIN IMAGE ACQUISITION AND DISPLAY LOOP ----
    while (remainingFrames--) {
        // Try to get an image frame (1 second timeout)
        iStatus = CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000);
        if (iStatus == CAMERA_STATUS_SUCCESS) {
            // Process the raw image frame to RGB/GRAY buffer
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer.get(), &sFrameInfo);

            // Wrap the image buffer in OpenCV Mat object without copying data
            Mat matImage(Size(sFrameInfo.iWidth, sFrameInfo.iHeight),
                         (channel == 1) ? CV_8UC1 : CV_8UC3, g_pRgbBuffer.get());

            // FPS calculation & update once per second
            frameCount++;
            auto now = std::chrono::high_resolution_clock::now();
            double elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
            if (elapsed_ms >= 1000.0) {
                fps = frameCount * 1000.0 / elapsed_ms;
                frameCount = 0;
                start = now;
            }

            // Overlay FPS counter on image
            char fps_text[32];
            sprintf(fps_text, "FPS: %.2f", fps);
            cv::putText(matImage, fps_text, cv::Point(20, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0,
                        (channel == 1) ? cv::Scalar(255) : cv::Scalar(0, 255, 0), 2);

            // Display the frame
            imshow("OpenCV Camera Demo", matImage);

            // Handle user key press events; 1 ms delay
            int key = waitKey(1);
            if (key == 27 || key == 'q') {  // Exit if ESC or 'q' pressed
                printf("Exit requested by user.\n");
                CameraReleaseImageBuffer(hCamera, pbyBuffer);
                break;
            }

            // Release the image buffer so the next frame can be grabbed
            CameraReleaseImageBuffer(hCamera, pbyBuffer);

        } else if (iStatus != CAMERA_STATUS_TIME_OUT) {
            // If an error besides timeout occurred, print it and exit loop
            std::cerr << "Failed to get image buffer. SDK error: " << iStatus << std::endl;
            break;
        }
        // If timeout occurs, continue trying to get frames
    }

    // ---- CLEANUP: UNINITIALIZE CAMERA ----
    CameraUnInit(hCamera);

    // Smart pointer automatically frees image buffer memory

    printf("Camera released and memory freed.\n");
    printf("Application finished successfully.\n");

    return 0;
}
// ---- END OF FILE ----
