#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# 📂 Define file paths for raw unsynchronized data
raw_files = {
    1: "raw_fiducial_data_cam1.csv",
    2: "raw_fiducial_data_cam2.csv",
    3: "raw_fiducial_data_cam3.csv"
}

# 🏗 Load raw data
raw_data = {cam: pd.read_csv(raw_files[cam]) for cam in raw_files}

# Extract timestamps (without synchronization, in milliseconds)
raw_timestamps = {cam: raw_data[cam]["Time (s)"].values * 1000 for cam in raw_data}

# 🔹 Apply manual correction for Camera 2 by shifting it back by 16.69 seconds (16690 ms)
raw_timestamps[2] -= 16690

# 🔹 Align all cameras to the earliest start time (Camera 3 starts earliest)
min_start_time = min([raw_timestamps[cam][0] for cam in raw_timestamps])
for cam in raw_timestamps:
    raw_timestamps[cam] -= min_start_time

# 🔹 Print adjusted timestamps per camera
for cam in raw_data:
    print(f"Camera {cam} Adjusted First Timestamp: {raw_timestamps[cam][0]:.2f} ms")

# 🔹 Detect Dropped Frames
frame_drop_counts = {}
plt.figure(figsize=(10, 5))
for cam in raw_timestamps:
    frame_intervals = np.diff(raw_timestamps[cam])
    mean_interval = np.mean(frame_intervals)
    threshold = mean_interval * 1.5  # If a frame interval is 1.5x the mean, it's a dropped frame
    dropped_frames = np.sum(frame_intervals > threshold)
    frame_drop_counts[cam] = dropped_frames
    plt.plot(frame_intervals, label=f"Cam {cam} Frame Interval")
plt.xlabel("Frame Index")
plt.ylabel("Time Between Frames (ms)")
plt.title("Frame Interval Variation Per Camera (Dropped Frame Detection)")
plt.legend()
plt.grid(True)
plt.show()

# 🔹 Match timestamps by finding the closest frame instead of interpolation
def find_nearest(reference_times, target_times):
    """ Matches each reference timestamp to the closest one in target timestamps. """
    matched_indices = np.searchsorted(target_times, reference_times)
    matched_indices = np.clip(matched_indices, 0, len(target_times) - 1)  # Ensure within bounds
    return target_times[matched_indices]

# Compute matched timestamps
drift_without_sync = {}
for cam1 in raw_timestamps:
    for cam2 in raw_timestamps:
        if cam1 < cam2:
            matched_times = find_nearest(raw_timestamps[cam1], raw_timestamps[cam2])
            drift_without_sync[(cam1, cam2)] = raw_timestamps[cam1] - matched_times

# 📊 Compute Time Offset Stability (Standard Deviation of Drift)
stability_without_sync = {pair: np.std(drift_without_sync[pair]) for pair in drift_without_sync}

# 📈 Plot Time Overlap Between Cameras
plt.figure(figsize=(12, 6))
for cam in raw_timestamps:
    plt.plot(raw_timestamps[cam], np.ones_like(raw_timestamps[cam]) * cam, '|', label=f"Cam {cam}")
plt.xlabel("Time (ms)")
plt.ylabel("Camera Number")
plt.title("Timestamp Overlap Between Cameras")
plt.legend()
plt.grid(True)
plt.show()

# 📝 Print Summary
print("📊 Unsynchronized Analysis Results:")
print(f"Inter-Camera Drift Without Sync (ms): {stability_without_sync}")
print("\n📊 Frame Drop Count Per Camera:")
for cam in frame_drop_counts:
    print(f"Camera {cam}: {frame_drop_counts[cam]} frames dropped")
 