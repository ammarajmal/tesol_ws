#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, resample, welch

# Experiment No. 1: Shaking Table - LDV Experiment
seconds_ = 10 # Duration of the experiment
fft_levels = 10
exp = 1
freq_ = 3
camera_ = 3
camera_data_path = "data_Exp1_10s_2024-09-14_06-45-54.csv"
ldv_data_path = "protocol_optoNCDT-ILD1420_2024-09-14_06-45-55.298.csv"
val = 44


# Load camera data
cam = pd.read_csv(camera_data_path)

# Load LDV data
ldv = pd.read_csv(ldv_data_path, skiprows=6, delimiter='\t')

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

# Create a new time axis for the resampled data
resampled_time = np.linspace(0, len(ldvd) / ldv_new_sampling_frequency, num_new_samples)

# Camera Interpolation and Resampling
time = cam['Time (s)'] - cam['Time (s)'].iloc[0]
data = cam[f'Cam{camera_} Position Y']

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
camd = pd.DataFrame({'Time (s)': time_resampled, f'Cam{camera_} Position Y': cam_resampled * 1e3})

# Trim the LDV data to match the length of the camera data
ldvd = ldvd[:len(camd)]
resampled_time = resampled_time[:len(camd)]


# *** Trim the data to only include the first 2 seconds ***
set_seconds_idx = np.where(resampled_time <= seconds_)[0]
ldvd = ldvd[set_seconds_idx]
camd = camd.iloc[set_seconds_idx]
resampled_time = resampled_time[set_seconds_idx]

# Find the maximum and minimum values for LDV and Camera data within 2 seconds
max_ldv = ldvd.max()
min_ldv = ldvd.min()
max_cam = camd[f'Cam{camera_} Position Y'].max()
min_cam = camd[f'Cam{camera_} Position Y'].min()

# Compute the difference between the max and min of LDV and Camera data
max_diff_signal = abs(max_ldv - max_cam)
min_diff_signal = abs(min_ldv - min_cam)



# Plot for the LDV and Camera data comparison
plt.figure(1)
plt.plot(resampled_time, ldvd, label=f'LDV Data (Max: {max_ldv:.2f}, Min: {min_ldv:.2f})')
plt.plot(resampled_time, camd[f'Cam{camera_} Position Y'], label=f'Camera Data (Max: {max_cam:.2f}, Min: {min_cam:.2f})')
plt.title(f'LDV and Camera Data Comparison - Cam {camera_}, Freq: {freq_} Hz')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.grid(True, which='both', linestyle='--')
plt.minorticks_on()  # Enable minor ticks to increase the grid density
plt.legend(loc='lower left')
plt.savefig(f'_LDV_camera_data_comparison_cam{camera_}_{freq_}Hz.png')
# Display the difference of max and min values on the plot using text
plt.text(0.05, 0.96, f'Difference in Max: {max_diff_signal:.3f}, Difference in Min: {min_diff_signal:.3f}', transform=plt.gca().transAxes, color='red')

# plt.show()


# FFT 
Nfft = 2**fft_levels
f_ldv, Pldv = welch(ldvd, fs=ldv_new_sampling_frequency, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)
f_cam, Pcam = welch(camd[f'Cam{camera_} Position Y'], fs=60, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)

# Find the maximum frequency components for LDV and Camera data
max_freq_ldv = f_ldv[np.argmax(Pldv)]
max_freq_cam = f_cam[np.argmax(Pcam)]

# Plotting the FFT
plt.figure(2)
plt.semilogy(f_ldv, Pldv, label=f'LDV Data (Max Frequency: {max_freq_ldv:.2f} Hz)')
plt.semilogy(f_cam, Pcam, label=f'Camera Data (Max Frequency: {max_freq_cam:.2f} Hz)')
plt.title(f'FFT of LDV and Camera Data - Cam {camera_}, Freq: {freq_} Hz')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Power Spectral Density')
plt.grid(True, which='both', linestyle='--')
plt.legend(loc='lower left')
plt.savefig(f'_FFT_LDV_camera_data_cam{camera_}_{freq_}Hz.png')
plt.show()
