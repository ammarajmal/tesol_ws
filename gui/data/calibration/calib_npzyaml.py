#!/usr/bin/env python3
import numpy as np
import yaml

# Load the calibration data from the .npz file
calibration_data = np.load('calibration_results.npz')

camera_matrix = calibration_data['camera_matrix']
dist_coeffs = calibration_data['dist_coeffs']
rvecs = calibration_data['rvecs']
tvecs = calibration_data['tvecs']

# Convert the data to the format required for the YAML file
yaml_data = {
    'image_width': 640,
    'image_height': 480,
    'camera_name': 'narrow_stereo',
    'camera_matrix': {
        'rows': 3,
        'cols': 3,
        'data': camera_matrix.flatten().tolist()
    },
    'distortion_model': 'plumb_bob',
    'distortion_coefficients': {
        'rows': 1,
        'cols': 5,
        'data': dist_coeffs.flatten().tolist()
    },
    'rectification_matrix': {
        'rows': 3,
        'cols': 3,
        'data': np.eye(3).flatten().tolist()  # Identity matrix
    },
    'projection_matrix': {
        'rows': 3,
        'cols': 4,
        'data': np.hstack((camera_matrix, np.zeros((3, 1)))).flatten().tolist()
    }
}

# Save the data into a YAML file
with open('calibration_results.yaml', 'w') as outfile:
    yaml.dump(yaml_data, outfile, default_flow_style=False)

print("Calibration data saved to calibration.yaml")
