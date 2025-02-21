#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft
from scipy.stats import linregress, ttest_rel, f_oneway, iqr

def load_data(raw_filename, sync_filename):
    """Loads raw and synchronized data from CSV files into Pandas DataFrames."""
    try:
        raw_df = pd.read_csv(raw_filename)
        sync_df = pd.read_csv(sync_filename)
        return raw_df, sync_df
    except FileNotFoundError:
        print(f"Error: File not found. Check filenames: {raw_filename}, {sync_filename}")
        return None, None
    except pd.errors.EmptyDataError:
        print(f"Error: CSV file is empty: {raw_filename} or {sync_filename}")
        return None, None
    except Exception as e:
        print(f"An error occurred loading data: {e}")
        return None, None

def calculate_time_offset(raw_df1, raw_df2, sync_df1, sync_df2):
    """Calculates time offset between two cameras using synchronized data."""
    if raw_df1 is None or raw_df2 is None or sync_df1 is None or sync_df2 is None:
        print("Error: One or more DataFrames are None. Cannot calculate time offset.")
        return [], []

    offsets = []
    raw_offsets = []
    for _, sync_row1 in sync_df1.iterrows():
        fiducial_id = sync_row1['Fiducial ID']
        raw_row1 = raw_df1[(raw_df1['System Time (s)'] >= sync_row1['Synchronized Time (s)'] - 0.001) &
                           (raw_df1['System Time (s)'] <= sync_row1['Synchronized Time (s)'] + 0.001) &
                           (raw_df1['Fiducial ID'] == fiducial_id)]
        raw_row2 = raw_df2[(raw_df2['System Time (s)'] >= sync_row1['Synchronized Time (s)'] - 0.001) &
                           (raw_df2['System Time (s)'] <= sync_row1['Synchronized Time (s)'] + 0.001) &
                           (raw_df2['Fiducial ID'] == fiducial_id)]
        if not raw_row1.empty and not raw_row2.empty:
            offset = (raw_row1['ROS Header Time (s)'].iloc[0] - raw_row2['ROS Header Time (s)'].iloc[0])*1000 #convert to ms
            offsets.append(offset)
            raw_offset = (raw_row1['ROS Header Time (s)'].iloc[0] - raw_row2['ROS Header Time (s)'].iloc[0])*1000 #convert to ms
            raw_offsets.append(raw_offset)
        else:
            print(f"Could not find raw data for cameras close to sync time {sync_row1['Synchronized Time (s)']} and fiducial id {fiducial_id}")
            #offsets.append(None) #DO NOT append None
            #raw_offsets.append(None) #DO NOT append None
            pass

    return offsets, raw_offsets

def calculate_metrics(offsets):
    """Calculates mean, std, max (abs), IQR, and drift rate of time offsets."""
    if not offsets:
        print("Warning: No offsets provided. Returning None for metrics.")
        return None, None, None, None, None

    offsets = [x for x in offsets if x is not None]
    if not offsets: #Added - check length after removing Nones
        print("Warning: No valid offsets provided. Returning None for metrics.")
        return None, None, None, None, None

    mean_offset = np.mean(offsets)
    std_offset = np.std(offsets)
    max_offset = np.max(np.abs(offsets))
    iqr_offset = iqr(offsets)
    time = np.arange(len(offsets))
    slope, _, _, _, _ = linregress(time, offsets)
    drift_rate = slope
    return mean_offset, std_offset, max_offset, iqr_offset, drift_rate

def plot_time_series(offsets, title):
    """Plots time series of time offsets."""
    if not offsets:
        print("Warning: No offsets to plot.")
        return

    plt.figure(figsize=(10, 6))
    plt.plot([x for x in offsets if x is not None])
    plt.title(title)
    plt.xlabel("Time (sample)")
    plt.ylabel("Time Offset (ms)") # correct unit
    plt.show()

def plot_distribution(data, title, xlabel):
    """Plots a distribution (histogram) of the given data."""
    if not data:
        print(f"Warning: No data to plot distribution for {title}.")
        return

    plt.figure(figsize=(10, 6))
    plt.hist([x for x in data if x is not None], bins=50)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel("Frequency")
    plt.show()

# File names (replace with your actual file names)
raw_file_cam1 = "raw_fiducial_data_cam1_2025-02-22_02-53-38.csv"
sync_file_cam1 = "sync_fiducial_data_cam1_2025-02-22_02-53-38.csv"
raw_file_cam2 = "raw_fiducial_data_cam2_2025-02-22_02-53-38.csv"
sync_file_cam2 = "sync_fiducial_data_cam2_2025-02-22_02-53-38.csv"
raw_file_cam3 = "raw_fiducial_data_cam3_2025-02-22_02-53-38.csv"
sync_file_cam3 = "sync_fiducial_data_cam3_2025-02-22_02-53-38.csv"

# Load the data
raw_df_cam1, sync_df_cam1 = load_data(raw_file_cam1, sync_file_cam1)
raw_df_cam2, sync_df_cam2 = load_data(raw_file_cam2, sync_file_cam2)
raw_df_cam3, sync_df_cam3 = load_data(raw_file_cam3, sync_file_cam3)

# Check all dataframes before running the code
if any(df is None for df in [raw_df_cam1, sync_df_cam1, raw_df_cam2, sync_df_cam2, raw_df_cam3, sync_df_cam3]):
  print("One or more dataframes failed to load. Exiting analysis.")
  exit()

# Analyze each pair of cameras

camera_pairs = [(1, 2), (1, 3), (2, 3)]

for cam1, cam2 in camera_pairs:
    print(f"\nAnalyzing Camera {cam1} vs Camera {cam2}:")

    # Construct dataframe names dynamically
    raw_df_cam1_pair = globals()[f'raw_df_cam{cam1}']
    sync_df_cam1_pair = globals()[f'sync_df_cam{cam1}']
    raw_df_cam2_pair = globals()[f'raw_df_cam{cam2}']
    sync_df_cam2_pair = globals()[f'sync_df_cam{cam2}']

    # Calculate time offsets
    offsets, raw_offsets = calculate_time_offset(raw_df_cam1_pair, raw_df_cam2_pair, sync_df_cam1_pair, sync_df_cam2_pair)

    if offsets:
        # Calculate metrics
        mean_offset, std_offset, max_offset, iqr_offset, drift_rate = calculate_metrics(offsets)
        mean_raw_offset, std_raw_offset, max_raw_offset, iqr_raw_offset, drift_rate_raw = calculate_metrics(raw_offsets)

        print(f"  Mean Time Offset (Sync): {mean_offset:.6f} ms")
        print(f"  Std Dev Time Offset (Sync): {std_offset:.6f} ms")
        print(f"  Max Time Offset (Sync): {max_offset:.6f} ms")
        print(f"  IQR Time Offset (Sync): {iqr_offset:.6f} ms")
        print(f"  Drift Rate (Sync): {drift_rate:.6f} ms/sample")

        print(f"  Mean Time Offset (Raw): {mean_raw_offset:.6f} ms")
        print(f"  Std Dev Time Offset (Raw): {std_raw_offset:.6f} ms")
        print(f"  Max Time Offset (Raw): {max_raw_offset:.6f} ms")
        print(f"  IQR Time Offset (Raw): {iqr_raw_offset:.6f} ms")
        print(f"  Drift Rate (Raw): {drift_rate_raw:.6f} ms/sample")

        # Plot distributions
        plot_distribution(offsets, f"Distribution of Time Offsets between Camera {cam1} and {cam2} (Synchronized)", "Time Offset (ms)")
        plot_distribution(raw_offsets, f"Distribution of Time Offsets between Camera {cam1} and {cam2} (Raw)", "Time Offset (ms)")

        # Plot time series
        plot_time_series(offsets, f"Time Offsets between Camera {cam1} and {cam2} (Synchronized)")
        plot_time_series(raw_offsets, f"Time Offsets between Camera {cam1} and {cam2} (Raw)")

        # Calculate and plot distribution of sync improvement
        improvement = [abs(r) - abs(s) for r, s in zip(raw_offsets, offsets) if r is not None and s is not None]
        plot_distribution(improvement, f"Distribution of Synchronization Improvement (Cam {cam1} vs Cam {cam2})", "Improvement (ms)")

    else:
        print(f"Could not compute offset between camera {cam1} and camera {cam2}")
