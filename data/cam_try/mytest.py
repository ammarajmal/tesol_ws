#! /usr/bin/env python3

desired_fps = 33.333
desired_fps = 50
CameraExposure = 1/(desired_fps * 1e-6)  # Convert to seconds
print(f"Using exposure time: {CameraExposure} us, Desired FPS: {desired_fps}")