#!/usr/bin/env python3

import numpy as np

R = np.array([[ 0.8473576,  -0.04119858,  0.52942211],
              [-0.0548275,  -0.9984452,   0.0100562 ],
              [ 0.52818466, -0.03754808, -0.84829895]])

R_inv = np.array([[ 0.8473576,  -0.0548275,   0.52818466],
                  [-0.04119858, -0.9984452,  -0.03754808],
                  [ 0.52942211,  0.0100562,  -0.84829895]])

# Compute the product
identity_matrix = np.dot(R, R_inv)

print("R * R_inv =\n", identity_matrix)
