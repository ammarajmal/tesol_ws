# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, resample, welch

# Experiment No. 1: Shaking Table - LDV Experiment
exp = 1
seconds_to_display = 'full'  # Duration of data to display in time domain plots (in seconds) or 'full' to display all data
camera_data_path = "data_Exp1_10s_2024-09-14_05-05-08.csv"
ldv_data_path = "protocol_optoNCDT-ILD1420_2024-09-14_05-05-08.710.csv"
val = 38
Nfft = 2**10

# Load camera data
cam = pd.read_csv(camera_data_path)

# Load LDV data
ldv = pd.read_csv(ldv_data_path, skiprows=6, delimiter=';')

# Strip the leading and trailing spaces from the column names
ldv.columns = ldv.columns.str.strip()

# Replacing commas with dots for numerical data
ldv['Epoch time (ms)'] = ldv['Epoch time (ms)'].str.replace(',', '.').astype(float)
ldv['Distance1 (mm)'] = ldv['Distance1 (mm)'].str.replace(',', '.').astype(float)

# Renaming the columns for easier reference
ldv.columns = ['acquisition_time', 'epoch_time_ms', 'distance_mm']

# LDV data processing
ldv['epoch_time'] = ldv['epoch_time_ms'] / 1000  # Convert to seconds

# Calculate the original sampling frequency
ldv_sampling_frequency = 1 / np.mean(np.diff(ldv['epoch_time']))

# Remove DC component from LDV data
ldv_dc_removed = ldv['distance_mm'] - np.mean(ldv['distance_mm'])

# Butterworth filter
b, a = butter(5, 20/500, btype='low')
ldvf = filtfilt(b, a, ldv_dc_removed.ravel())  # Filter the data

# Set the new sampling frequency
ldv_new_sampling_frequency = 60

# Calculate the number of samples for the new sampling frequency
num_original_samples = len(ldvf)
num_new_samples = int(num_original_samples * ldv_new_sampling_frequency / ldv_sampling_frequency)

# Resample the data to the new sampling frequency
ldvd = resample(ldvf, num_new_samples)
data_length = len(ldvd)

# Create a new time axis for the resampled data
resampled_time = np.linspace(0, len(ldvd) / ldv_new_sampling_frequency, num_new_samples)

# Camera Interpolation and Resampling
time = cam['Time (s)'] - cam['Time (s)'].iloc[0]
data = cam['Cam2 Position Y']

# Remove DC component from Camera data
data_dc_removed = data - np.mean(data)

# Interpolate camera data to 1 KHz
time_new = np.arange(0, time.iloc[-1], 1e-3)
datau = np.interp(time_new, time, data_dc_removed)

# Resample the camera data to 60 Hz
cam_resampled = resample(datau, 60 * len(datau) // 1000)
cam_resampled = cam_resampled[val:]

# Time vector for resampled data
time_resampled = np.linspace(0, len(cam_resampled) / 60, len(cam_resampled))
camd = pd.DataFrame({'Time (s)': time_resampled, 'Cam2 Position Y': cam_resampled * 1e3})

# Trim the LDV data to match the length of the camera data
ldvd = ldvd[:len(camd)]
resampled_time = resampled_time[:len(camd)]


# *** Check if 'seconds_to_display' is numeric or 'full' ***
if isinstance(seconds_to_display, (int, float)):  # If seconds_to_display is a numeric value
    # Trim the data to only include the first 'seconds_to_display' seconds for time-domain plot
    set_seconds_idx = np.where(resampled_time <= seconds_to_display)[0]
    ldvd_time_plot = ldvd[set_seconds_idx]  # LDV data for time plot
    camd_time_plot = camd.iloc[set_seconds_idx]  # Camera data for time plot
    resampled_time_plot = resampled_time[set_seconds_idx]  # Time vector for the trimmed data
elif seconds_to_display == 'full':  # If 'seconds_to_display' is set to 'full', display all data
    ldvd_time_plot = ldvd
    camd_time_plot = camd
    resampled_time_plot = resampled_time
else:
    raise ValueError("Invalid value for 'seconds_to_display'. It should be either a numeric value or 'full'.")

# Find the maximum and minimum values for LDV and Camera data within the selected duration
max_ldv = ldvd_time_plot.max()
min_ldv = ldvd_time_plot.min()
max_cam = camd_time_plot['Cam2 Position Y'].max()
min_cam = camd_time_plot['Cam2 Position Y'].min()

# Compute the difference between the max and min of LDV and Camera data
max_diff_signal = abs(max_ldv - max_cam)
min_diff_signal = abs(min_ldv - min_cam)

# Plot for the LDV and Camera data comparison (Time domain)
plt.figure(1)
plt.plot(resampled_time_plot, ldvd_time_plot, label=f'LDV Data (Max: {max_ldv:.2f}, Min: {min_ldv:.2f})')
plt.plot(resampled_time_plot, camd_time_plot['Cam2 Position Y'].values, label=f'Camera Data (Max: {max_cam:.2f}, Min: {min_cam:.2f})')
plt.title(f'LDV and Camera Data Comparison (Time Domain)')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.grid(True, which='both', linestyle='--')
plt.minorticks_on()  # Enable minor ticks to increase the grid density
plt.legend(loc='lower left')
plt.savefig(f'_LDV_camera_data_comparison_time_domain.png')
# Display the difference of max and min values on the plot using text
plt.text(0.05, 0.96, f'Difference in Max: {max_diff_signal:.2f}, Difference in Min: {min_diff_signal:.2f}', transform=plt.gca().transAxes, color='red')

# FFT (Use all data for frequency domain plot)

nperseg = min(Nfft // 2, data_length)
noverlap = min(Nfft // 4, nperseg - 1)

# Frequency domain analysis using all the data
f_ldv, Pldv = welch(ldvd, fs=ldv_new_sampling_frequency, nperseg=nperseg, noverlap=noverlap, nfft=Nfft)
f_cam, Pcam = welch(camd['Cam2 Position Y'], fs=60, nperseg=nperseg, noverlap=noverlap, nfft=Nfft)

# Find the maximum frequency components for LDV and Camera data
max_freq_ldv = f_ldv[np.argmax(Pldv)]
max_freq_cam = f_cam[np.argmax(Pcam)]

# Plotting the FFT (Frequency domain)
plt.figure(2)
plt.semilogy(f_ldv, Pldv, label=f'LDV Data (Max Frequency: {max_freq_ldv:.2f} Hz)')
plt.semilogy(f_cam, Pcam, label=f'Camera Data (Max Frequency: {max_freq_cam:.2f} Hz)')
plt.title(f'FFT of LDV and Camera Data')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Power Spectral Density')
plt.grid(True, which='both', linestyle='--')
plt.legend(loc='lower left')
plt.savefig(f'_FFT_LDV_camera_data.png')
plt.show()
