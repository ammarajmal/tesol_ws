import numpy as np
from scipy.spatial.transform import Rotation as R

# Method 1: Using scipy
def rotation_matrix_to_quaternion_scipy(rotation_matrix):
    r = R.from_matrix(rotation_matrix)
    quaternion = r.as_quat()  # Returns (x, y, z, w)
    return quaternion

# Method 2: Using custom implementation
def rotation_matrix_to_quaternion_custom(R):
    q = np.empty((4,))
    t = np.trace(R)
    if t > 0:
        S = np.sqrt(t + 1.0) * 2
        q[3] = 0.25 * S
        q[0] = (R[2, 1] - R[1, 2]) / S
        q[1] = (R[0, 2] - R[2, 0]) / S
        q[2] = (R[1, 0] - R[0, 1]) / S
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            q[3] = (R[2, 1] - R[1, 2]) / S
            q[0] = 0.25 * S
            q[1] = (R[0, 1] + R[1, 0]) / S
            q[2] = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            q[3] = (R[0, 2] - R[2, 0]) / S
            q[0] = (R[0, 1] + R[1, 0]) / S
            q[1] = 0.25 * S
            q[2] = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            q[3] = (R[1, 0] - R[0, 1]) / S
            q[0] = (R[0, 2] + R[2, 0]) / S
            q[1] = (R[1, 2] + R[2, 1]) / S
    return q

# Test rotation matrix
rotation_matrix = np.array([[0.36, 0.48, -0.8],
                            [-0.8, 0.60, 0],
                            [0.48, 0.64, 0.6]])

# Get quaternions from both methods
quaternion_scipy = rotation_matrix_to_quaternion_scipy(rotation_matrix)
quaternion_custom = rotation_matrix_to_quaternion_custom(rotation_matrix)

# Print results
print("Quaternion (scipy):", quaternion_scipy)
print("Quaternion (custom):", quaternion_custom)
import time

# Number of iterations
iterations = 100000

# Timing scipy implementation
start_time = time.time()
for _ in range(iterations):
    quaternion_scipy = rotation_matrix_to_quaternion_scipy(rotation_matrix)
scipy_duration = time.time() - start_time

# Timing custom implementation
start_time = time.time()
for _ in range(iterations):
    quaternion_custom = rotation_matrix_to_quaternion_custom(rotation_matrix)
custom_duration = time.time() - start_time

# Print timing results
print(f"Scipy duration: {scipy_duration:.6f} seconds")
print(f"Custom duration: {custom_duration:.6f} seconds")
