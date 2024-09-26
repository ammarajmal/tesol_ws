#!/usr/env/bin python3
import cv2
import numpy as np
import os
import yaml

# Load camera parameters from YAML file
def load_camera_parameters(file_path):
    with open(file_path, 'r') as f:
        camera_data = yaml.safe_load(f)
    camera_matrix = np.array(camera_data['camera_matrix']['data']).reshape((3, 3))
    dist_coeffs = np.array(camera_data['distortion_coefficients']['data'])
    return camera_matrix, dist_coeffs

# Detect Charuco board in the image
def detect_charuco(image, dictionary, charuco_board):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = cv2.aruco.detectMarkers(gray_image, dictionary)
    if len(corners) > 0:
        _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray_image, charuco_board
        )
        return charuco_corners, charuco_ids
    return None, None

# Save the refined calibration in a specific format
def save_calibration(file_path, image_width, image_height, camera_matrix, dist_coeffs, projection_matrix, rectification_matrix):
    calibration_data = {
        'image_width': image_width,
        'image_height': image_height,
        'distortion_model': 'rational_polynomial',
        'camera_name': 'camera',
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': camera_matrix.flatten().tolist()
        },
        'distortion_coefficients': {
            'rows': 1,
            'cols': 14,  # You can modify the number of coefficients as per the model you use
            'data': dist_coeffs.flatten().tolist()
        },
        'rectification_matrix': {
            'rows': 3,
            'cols': 3,
            'data': rectification_matrix.flatten().tolist()
        },
        'projection_matrix': {
            'rows': 3,
            'cols': 4,
            'data': projection_matrix.flatten().tolist()
        }
    }
    with open(file_path, 'w') as f:
        yaml.dump(calibration_data, f)

# Perform camera calibration refinement using Charuco board images
def refine_calibration(image_dir, camera_matrix, dist_coeffs, charuco_board, dictionary, image_width, image_height):
    obj_points = []  # Real world 3D points (board corners)
    img_points = []  # Image points (detected corners)
    img_ids = []     # Detected corner IDs
    valid_images = 0

    # Iterate over images in the directory
    for filename in sorted(os.listdir(image_dir)):
        if filename.endswith(".png"):
            print(f"Processing {filename}")
            image = cv2.imread(os.path.join(image_dir, filename))
            if image is None:
                print(f"Failed to load image {filename}")
                continue

            charuco_corners, charuco_ids = detect_charuco(image, dictionary, charuco_board)

            if charuco_corners is not None and len(charuco_corners) == 28:  # Use only images with all 28 corners
                print(f"Charuco board detected in {filename}, {len(charuco_corners)} corners detected")
                img_points.append(charuco_corners)
                img_ids.append(charuco_ids)  # Keep track of the corresponding IDs
                valid_images += 1

                # Draw detected corners for visual feedback
                cv2.aruco.drawDetectedCornersCharuco(image, charuco_corners, charuco_ids)
                cv2.imshow('Charuco Detection', image)
                cv2.waitKey(100)
            else:
                print(f"Skipping {filename}: insufficient corners detected.")

    if len(img_points) == 0 or len(img_ids) == 0:
        print("No sufficient Charuco corners detected for calibration. Exiting.")
        return

    print(f"Total valid images used for calibration: {valid_images}")
    
    # Refine camera parameters
    try:
        ret, new_camera_matrix, new_dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
            charucoCorners=img_points,
            charucoIds=img_ids,   # Use the IDs here instead of repeating corners
            board=charuco_board,
            imageSize=(image_width, image_height),
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
            flags=cv2.CALIB_USE_INTRINSIC_GUESS
        )

        # Create rectification and projection matrices (for stereo, we'll use identity)
        rectification_matrix = np.eye(3)
        projection_matrix = np.zeros((3, 4))
        projection_matrix[:3, :3] = new_camera_matrix

        # Save refined parameters to a new file with the requested format
        save_calibration('refined_calibration.yaml', image_width, image_height, new_camera_matrix, new_dist_coeffs, projection_matrix, rectification_matrix)

        print("Refined calibration parameters saved to refined_calibration.yaml")
    except cv2.error as e:
        print(f"Calibration failed: {e}")

if __name__ == '__main__':
    # Parameters
    image_dir = './'  # Directory containing the images
    calibration_file = 'calibration.yaml'  # Existing camera calibration parameters

    # Load the existing camera calibration
    camera_matrix, dist_coeffs = load_camera_parameters(calibration_file)

    # Define the Charuco board parameters (from your launch file)
    squares_x = 8  # Number of squares in the X direction
    squares_y = 5  # Number of squares in the Y direction
    square_length = 0.006  # Size of a square in meters (6mm)
    marker_length = 0.004  # Size of the ArUco marker in meters (4mm)

    # Set image resolution
    image_width = 1280
    image_height = 720

    # Create a Charuco board and dictionary
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
    charuco_board = cv2.aruco.CharucoBoard_create(
        squares_x, squares_y, square_length, marker_length, dictionary
    )

    # Refine the calibration
    refine_calibration(image_dir, camera_matrix, dist_coeffs, charuco_board, dictionary, image_width, image_height)
