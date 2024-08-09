import cv2
import numpy as np
import glob

# Define the checkerboard dimensions
CHECKERBOARD = (8, 10)  # (width-1, height-1) corners
square_size = 0.00633  # Size of a square in your defined unit (e.g., meters)

# Termination criteria for corner subpixel refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points based on the actual checkerboard dimensions
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Get the list of images in the current directory
images = glob.glob('*.png')  # Change extension as needed (e.g., *.png)



for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the checkerboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    # If found, add object points and image points (after refining them)
    if ret:
        objpoints.append(objp)
        refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(refined_corners)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, CHECKERBOARD, refined_corners, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Perform camera calibration to obtain the camera matrix, distortion coefficients, etc.
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Print the results
print("Camera matrix:\n", camera_matrix)
print("Distortion coefficients:\n", dist_coeffs)
print("Rotation vectors:\n", rvecs)
print("Translation vectors:\n", tvecs)

# Optionally, save the results to a file
np.savez('calibration_results.npz', camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, rvecs=rvecs, tvecs=tvecs)

# To undistort an image using the calibration results:
for fname in images:
    img = cv2.imread(fname)
    h, w = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))

    # Undistort the image
    undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)

    # Crop the image based on the region of interest (roi)
    x, y, w, h = roi
    undistorted_img = undistorted_img[y:y+h, x:x+w]

    # Display and save the undistorted image
    cv2.imshow('Undistorted Image', undistorted_img)
    cv2.imwrite('undistorted_' + fname, undistorted_img)
    cv2.waitKey(500)

cv2.destroyAllWindows()
