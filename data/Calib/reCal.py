#!/usr/bin/env python3

import numpy as np

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
print("Updated camera matrix:")
print(camera_matrix)
