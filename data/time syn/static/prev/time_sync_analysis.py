#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load CSV files
raw_data = pd.read_csv("raw_fiducial_data_2025-02-22_01-38-11.csv")
sync_data = pd.read_csv("sync_fiducial_data_2025-02-22_01-38-11.csv")

# Extract timestamps
raw_timestamps = raw_data['ROS Header Time (s)']
sync_timestamps = sync_data['ROS Header Time (s)']

# Compute time drift (differences between successive timestamps)
raw_drift = raw_timestamps.diff().dropna().to_numpy()
sync_drift = sync_timestamps.diff().dropna().to_numpy()

# Compute inter-camera time differences
raw_inter_cam_diff = raw_timestamps - raw_timestamps.iloc[0]
sync_inter_cam_diff = sync_timestamps - sync_timestamps.iloc[0]

# Compute time jitter (standard deviation of drift)
raw_jitter = np.std(raw_drift)
sync_jitter = np.std(sync_drift)

# Compute synchronization improvement
sync_improvement = raw_inter_cam_diff - sync_inter_cam_diff

# Generate visual plots

# Time Drift Distribution
plt.figure(figsize=(10, 5))
plt.hist(raw_drift, bins=50, alpha=0.5, label="Raw Time Drift", color='red')
plt.hist(sync_drift, bins=50, alpha=0.5, label="Sync Time Drift", color='blue')
plt.xlabel("Time Drift (s)")
plt.ylabel("Frequency")
plt.title("Time Drift Distribution Before and After Synchronization")
plt.legend()
plt.grid(True)
plt.show()

# Synchronization Improvement Plot
plt.figure(figsize=(10, 5))
plt.plot(raw_inter_cam_diff, label="Raw Inter-Camera Time Difference", color='red', linestyle='--')
plt.plot(sync_inter_cam_diff, label="Sync Inter-Camera Time Difference", color='blue')
plt.xlabel("Frame Index")
plt.ylabel("Time Difference (s)")
plt.title("Inter-Camera Time Differences Before and After Synchronization")
plt.legend()
plt.grid(True)
plt.show()

# Time Jitter Before and After Synchronization
plt.figure(figsize=(10, 5))
plt.bar(["Raw Jitter", "Sync Jitter"], [raw_jitter, sync_jitter], color=['red', 'blue'])
plt.ylabel("Jitter (s)")
plt.title("Time Jitter Before and After Synchronization")
plt.grid(True)
plt.show()

# Synchronization Improvement Histogram
plt.figure(figsize=(10, 5))
plt.hist(sync_improvement, bins=50, alpha=0.7, color='green', label="Sync Improvement")
plt.xlabel("Synchronization Improvement (s)")
plt.ylabel("Frequency")
plt.title("Distribution of Synchronization Improvement")
plt.legend()
plt.grid(True)
plt.show()
