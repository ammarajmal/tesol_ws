#!/usr/bin/env python3

import numpy as np
import yaml

# Known parameters
W = 40  # AprilTag size in mm
Z = 2400  # Distance to tag in mm
w = 150  # Detected size of AprilTag in pixels (example value)

# Calculate focal length
f = (w * Z) / W
print(f"Estimated focal length: {f} pixels")

# Image resolution
image_width = 640  # Example resolution width
image_height = 480  # Example resolution height

# Update camera intrinsic matrix
camera_matrix = np.array([
    [f, 0, image_width / 2],
    [0, f, image_height / 2],
    [0, 0, 1]
])

# Distortion coefficients (initialize to zero if not recalibrating)
distortion_coefficients = np.zeros(14)

# Rectification matrix
rectification_matrix = np.eye(3)

# Projection matrix
projection_matrix = np.zeros((3, 4))
projection_matrix[:3, :3] = camera_matrix

# Create the YAML structure
camera_params = {
    "image_width": image_width,
    "image_height": image_height,
    "distortion_model": "rational_polynomial",
    "camera_name": "sony_cam2",
    "camera_matrix": {
        "rows": 3,
        "cols": 3,
        "data": camera_matrix.flatten().tolist(),
    },
    "distortion_coefficients": {
        "rows": 1,
        "cols": 14,
        "data": distortion_coefficients.tolist(),
    },
    "rectification_matrix": {
        "rows": 3,
        "cols": 3,
        "data": rectification_matrix.flatten().tolist(),
    },
    "projection_matrix": {
        "rows": 3,
        "cols": 4,
        "data": projection_matrix.flatten().tolist(),
    },
}

# Save to a YAML file
output_file = "sony_cam2_info.yaml"
with open(output_file, "w") as file:
    yaml.dump(camera_params, file, default_flow_style=False)

print(f"Camera parameters saved to {output_file}")
