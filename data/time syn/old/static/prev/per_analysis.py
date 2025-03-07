#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft
from scipy.stats import linregress, ttest_rel, f_oneway

def load_data(raw_filename, sync_filename):
    """Loads raw and synchronized data from CSV files into Pandas DataFrames.

    Args:
        raw_filename (str): Path to the raw data CSV file.
        sync_filename (str): Path to the synchronized data CSV file.

    Returns:
        tuple: A tuple containing two Pandas DataFrames: (raw_df, sync_df).
               Returns (None, None) if either file is not found or has an error.
    """
    try:
        raw_df = pd.read_csv(raw_filename)
        sync_df = pd.read_csv(sync_filename)
        return raw_df, sync_df
    except FileNotFoundError:
        print(f"Error: File not found.  Check filenames: {raw_filename}, {sync_filename}")
        return None, None
    except pd.errors.EmptyDataError:
        print(f"Error: CSV file is empty: {raw_filename} or {sync_filename}")
        return None, None
    except Exception as e:
        print(f"An error occurred loading data: {e}")  # Catch other potential errors
        return None, None


def calculate_time_offset(raw_df1, raw_df2, sync_df1, sync_df2):
    """Calculates time offset between two cameras using synchronized data.

    Args:
        raw_df1 (pd.DataFrame): Raw data for camera 1.
        raw_df2 (pd.DataFrame): Raw data for camera 2.
        sync_df1 (pd.DataFrame): Synchronized data for camera 1.
        sync_df2 (pd.DataFrame): Synchronized data for camera 2.

    Returns:
        list: A list of time offsets (in seconds) between the two cameras.
              Returns an empty list if any of the DataFrames are None.
    """

    if raw_df1 is None or raw_df2 is None or sync_df1 is None or sync_df2 is None:
        print("Error: One or more DataFrames are None. Cannot calculate time offset.")
        return []

    offsets = []
    for _, sync_row1 in sync_df1.iterrows():
        # Get the fiducial ID from the synchronized data
        fiducial_id = sync_row1['Fiducial ID']

        # Find the corresponding raw entries for both cameras
        raw_row1 = raw_df1[(raw_df1['System Time (s)'] >= sync_row1['Synchronized Time (s)'] - 0.001) &
                           (raw_df1['System Time (s)'] <= sync_row1['Synchronized Time (s)'] + 0.001) &
                           (raw_df1['Fiducial ID'] == fiducial_id)]
        raw_row2 = raw_df2[(raw_df2['System Time (s)'] >= sync_row1['Synchronized Time (s)'] - 0.001) &
                           (raw_df2['System Time (s)'] <= sync_row1['Synchronized Time (s)'] + 0.001) &
                           (raw_df2['Fiducial ID'] == fiducial_id)]
        if not raw_row1.empty and not raw_row2.empty:
          # Calculate the offset using the *ROS Header Time*
          offset = raw_row1['ROS Header Time (s)'].iloc[0] - raw_row2['ROS Header Time (s)'].iloc[0]
          offsets.append(offset)
        elif raw_row1.empty:
            print(f"Could not find raw data for camera 1 close to sync time {sync_row1['Synchronized Time (s)'] } and fiducial id {fiducial_id}")
        elif raw_row2.empty:
            print(f"Could not find raw data for camera 2 close to sync time {sync_row1['Synchronized Time (s)'] } and fiducial id {fiducial_id}")

    return offsets


def calculate_metrics(offsets):
    """Calculates mean, std, max (abs), and drift rate of time offsets.

    Args:
        offsets (list): A list of time offsets.

    Returns:
        tuple: A tuple containing mean offset, std offset, max offset, drift rate.
    """
    if not offsets:
        print("Warning: No offsets provided. Returning None for metrics.")
        return None, None, None, None

    mean_offset = np.mean(offsets)
    std_offset = np.std(offsets)
    max_offset = np.max(np.abs(offsets))
    # Drift rate (linear regression)
    time = np.arange(len(offsets))
    slope, intercept, r_value, p_value, std_err = linregress(time, offsets)
    drift_rate = slope
    return mean_offset, std_offset, max_offset, drift_rate


def plot_time_series(offsets, title):
    """Plots time series of time offsets.

    Args:
        offsets (list): A list of time offsets.
        title (str): The title of the plot.
    """
    if not offsets:
        print("Warning: No offsets to plot.")
        return

    plt.figure(figsize=(10, 6))
    plt.plot(offsets)
    plt.title(title)
    plt.xlabel("Time (sample)")
    plt.ylabel("Time Offset (s)")
    plt.show()


def perform_fft(data, sampling_rate):
    """Performs FFT analysis.  Assumes evenly sampled data.

    Args:
        data (list): Time series data.
        sampling_rate (float): The sampling rate of the data (samples per second).

    Returns:
        tuple: Frequencies and magnitudes of the FFT.
    """
    if not data:
        print("Warning: No data for FFT.")
        return [], []

    N = len(data)
    yf = fft(data)
    xf = np.fft.fftfreq(N, 1 / sampling_rate)
    return xf[:N//2], np.abs(yf[:N//2]) # Return freqs and magnitudes


def compare_to_ground_truth(estimated, ground_truth):
    """Calculates RMSE and correlation coefficient.

    Args:
        estimated (list): Estimated values.
        ground_truth (list): Ground truth values.

    Returns:
        tuple: RMSE and correlation coefficient.
    """
    if not estimated or not ground_truth:
        print("Warning: No data for ground truth comparison.")
        return None, None

    rmse = np.sqrt(np.mean((np.array(estimated) - np.array(ground_truth))**2))
    correlation = np.corrcoef(estimated, ground_truth)[0, 1]
    return rmse, correlation


# File names (replace with your actual file names)
raw_file_cam2 = "raw_fiducial_data_cam2_2025-02-22_02-32-27.csv"
sync_file_cam2 = "sync_fiducial_data_cam2_2025-02-22_02-32-27.csv"
raw_file_cam3 = "raw_fiducial_data_cam3_2025-02-22_02-32-27.csv"
sync_file_cam3 = "sync_fiducial_data_cam3_2025-02-22_02-32-27.csv"

# Load the data
raw_df_cam2, sync_df_cam2 = load_data(raw_file_cam2, sync_file_cam2)
raw_df_cam3, sync_df_cam3 = load_data(raw_file_cam3, sync_file_cam3)


# Calculate time offsets between Cam2 and Cam3
offsets_2_3 = calculate_time_offset(raw_df_cam2, raw_df_cam3, sync_df_cam2, sync_df_cam3)
if offsets_2_3:
    # Calculate metrics
    mean_offset_2_3, std_offset_2_3, max_offset_2_3, drift_rate_2_3 = calculate_metrics(offsets_2_3)

    print("Camera 2 vs Camera 3:")
    print(f"  Mean Time Offset: {mean_offset_2_3:.6f} s")
    print(f"  Std Dev Time Offset: {std_offset_2_3:.6f} s")
    print(f"  Max Time Offset: {max_offset_2_3:.6f} s")
    print(f"  Drift Rate: {drift_rate_2_3:.6f} s/sample")

    # Plot time series
    plot_time_series(offsets_2_3, "Time Offsets between Camera 2 and 3 (Synchronized)")

else:
    print("Could not compute offset between camera 2 and camera 3")


# FFT analysis and ground truth comparison would go here, if you have that data.
