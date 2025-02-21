#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# 📂 Define file paths (update with actual filenames)
raw_files = {
    1: "raw_fiducial_data_cam1_2025-02-22_04-41-40.csv",
    2: "raw_fiducial_data_cam2_2025-02-22_04-41-40.csv",
    3: "raw_fiducial_data_cam3_2025-02-22_04-41-40.csv"
}
sync_files = {
    1: "sync_fiducial_data_cam1_2025-02-22_04-41-40.csv",
    2: "sync_fiducial_data_cam2_2025-02-22_04-41-40.csv",
    3: "sync_fiducial_data_cam3_2025-02-22_04-41-40.csv"
}

# 🏗 Load raw data
raw_data = {cam: pd.read_csv(raw_files[cam]) for cam in raw_files}
sync_data = {cam: pd.read_csv(sync_files[cam]) for cam in sync_files}

# Convert timestamps to milliseconds
raw_timestamps = {cam: raw_data[cam]["ROS Header Time (s)"].values * 1000 for cam in raw_data}
sync_timestamps = {cam: sync_data[cam]["Synchronized Time (s)"].values * 1000 for cam in sync_data}

# 🔹 Ensure timestamps have the same shape by interpolating
def interpolate_timestamps(reference, target):
    """ Interpolates the target timestamps to match the reference timestamps """
    return np.interp(reference, target, target)

# Find the camera with the most timestamps as reference
reference_cam = max(raw_timestamps, key=lambda cam: len(raw_timestamps[cam]))
reference_timestamps = raw_timestamps[reference_cam]

# Interpolate other camera timestamps
aligned_timestamps = {
    cam: interpolate_timestamps(reference_timestamps, raw_timestamps[cam]) for cam in raw_timestamps
}

# 📊 Compute Inter-Camera Drift (Before Sync)
drift_before = {
    (1, 2): aligned_timestamps[1] - aligned_timestamps[2],
    (1, 3): aligned_timestamps[1] - aligned_timestamps[3],
    (2, 3): aligned_timestamps[2] - aligned_timestamps[3]
}

# 📊 Compute Synchronization Accuracy (After Sync)
aligned_sync_timestamps = {
    cam: interpolate_timestamps(reference_timestamps, sync_timestamps[cam]) for cam in sync_timestamps
}
drift_after = {
    (1, 2): aligned_sync_timestamps[1] - aligned_sync_timestamps[2],
    (1, 3): aligned_sync_timestamps[1] - aligned_sync_timestamps[3],
    (2, 3): aligned_sync_timestamps[2] - aligned_sync_timestamps[3]
}

# 📊 Compute Synchronization Latency
latency = np.max(np.stack(list(aligned_sync_timestamps.values())), axis=0) - np.min(np.stack(list(aligned_sync_timestamps.values())), axis=0)

# 📊 Compute Frame Drop Rate
raw_frame_count = {cam: len(raw_timestamps[cam]) for cam in raw_timestamps}
sync_frame_count = {cam: len(sync_timestamps[cam]) for cam in sync_timestamps}
frame_drop_rate = {cam: (1 - sync_frame_count[cam] / raw_frame_count[cam]) * 100 for cam in raw_frame_count}

# 📊 Compute Time Offset Stability (Standard Deviation of Drift)
stability_before = {pair: np.std(drift_before[pair]) for pair in drift_before}
stability_after = {pair: np.std(drift_after[pair]) for pair in drift_after}

# 📈 Plot Results
plt.figure(figsize=(12, 6))
plt.hist(drift_before[(1, 2)], bins=50, alpha=0.6, label="Before Sync")
plt.hist(drift_after[(1, 2)], bins=50, alpha=0.6, label="After Sync")
plt.xlabel("Time Drift (ms)")
plt.ylabel("Frequency")
plt.title("Inter-Camera Time Drift")
plt.legend()
plt.grid(True)
plt.show()

# 📝 Print Summary
print("📊 Synchronization Analysis Results:")
print(f"Inter-Camera Drift Before Sync (ms): {stability_before}")
print(f"Inter-Camera Drift After Sync (ms): {stability_after}")
print(f"Synchronization Latency (ms): {np.mean(latency):.2f} ± {np.std(latency):.2f}")
print(f"Frame Drop Rate (%): {frame_drop_rate}")
