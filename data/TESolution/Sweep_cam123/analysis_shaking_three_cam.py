#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, resample, welch

# Experiment Details
exp = 9
camera_data_path = "sweep123_120s_2024-11-20_02-13-43.csv"
ldv_data_path = "protocol_optoNCDT-ILD1420_2024-11-20_02-13-43.472.csv"
DELAY = 37  # Delay in seconds to align LDV and Camera data
plot_duration = 3  # Set time duration (in seconds) for time-domain plot

# Load Camera Data
cam = pd.read_csv(camera_data_path)

# Load LDV Data
ldv = pd.read_csv(ldv_data_path, skiprows=6, delimiter=';')
ldv.columns = ldv.columns.str.strip()  # Strip leading/trailing spaces from column names

# LDV Data Processing
ldv['Epoch time (ms)'] = ldv['Epoch time (ms)'].astype(float)
ldv['Distance1 (mm)'] = ldv['Distance1 (mm)'].astype(float)
ldv['epoch_time'] = ldv['Epoch time (ms)'] / 1000  # Convert to seconds
ldv_sampling_frequency = 1 / np.mean(np.diff(ldv['epoch_time']))
ldv_dc_removed = ldv['Distance1 (mm)'] - np.mean(ldv['Distance1 (mm)'])

# Apply Butterworth filter
b, a = butter(5, 20 / 500, btype='low')
ldvf = filtfilt(b, a, ldv_dc_removed.ravel())

# Resample LDV data to 60 Hz
ldv_new_sampling_frequency = 60
num_original_samples = len(ldvf)
num_new_samples = int(num_original_samples * ldv_new_sampling_frequency / ldv_sampling_frequency)
ldvd = resample(ldvf, num_new_samples)
resampled_time = np.linspace(0, len(ldvd) / ldv_new_sampling_frequency, num_new_samples)

# Camera Data Processing for Three Cameras
time = cam['Time (s)'] - cam['Time (s)'].iloc[0]
camera_data = cam[['Cam1 Position X', 'Cam2 Position X', 'Cam3 Position X']].values

# Remove DC Component from Camera data
camera_dc_removed = camera_data - np.mean(camera_data, axis=0)

# Interpolate Camera data to 1 kHz
time_new = np.arange(0, time.iloc[-1], 1e-3)
camera_interpolated = np.zeros((len(time_new), 3))
for i in range(3):
    camera_interpolated[:, i] = np.interp(time_new, time, camera_dc_removed[:, i])

# Resample Camera data to 60 Hz
camera_resampled = resample(camera_interpolated, 60 * len(camera_interpolated) // 1000)
camera_resampled = camera_resampled[DELAY:]  # Remove initial delay samples
time_resampled = np.linspace(0, len(camera_resampled) / 60, len(camera_resampled))

# Create DataFrame for Resampled Camera Data
camera_df = pd.DataFrame({
    'Time (s)': time_resampled,
    'Cam1 Position X': camera_resampled[:, 0] * 1e3,
    'Cam2 Position X': camera_resampled[:, 1] * 1e3,
    'Cam3 Position X': camera_resampled[:, 2] * 1e3,
})

# Trim LDV data to match Camera data length
ldvd = ldvd[:len(camera_df)]
resampled_time = resampled_time[:len(camera_df)]

# Plot for LDV and Camera data comparison (Time-Domain)
plt.figure(1)
plt.plot(resampled_time, ldvd, label='LDV Data')
plt.plot(resampled_time, -camera_df['Cam1 Position X'].to_numpy(), label='Cam1 Data')
plt.plot(resampled_time, -camera_df['Cam2 Position X'].to_numpy(), label='Cam2 Data')
plt.plot(resampled_time, -camera_df['Cam3 Position X'].to_numpy(), label='Cam3 Data')
plt.title(f'LDV and Camera Data Comparison (Time Domain - {plot_duration} Seconds)')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude (mm)')
plt.legend()
plt.grid()
plt.savefig('Time_Domain_Comparison_Three_Cameras.png')

# Frequency-Domain Analysis
fft_levels = 12
Nfft = 2 ** fft_levels

# Welch's method for LDV
f_ldv, Pldv = welch(ldvd, fs=ldv_new_sampling_frequency, nperseg=Nfft // 2, noverlap=Nfft // 4, nfft=Nfft)

# Welch's method for Cameras
f_cam1, Pcam1 = welch(camera_df['Cam1 Position X'], fs=60, nperseg=Nfft // 2, noverlap=Nfft // 4, nfft=Nfft)
f_cam2, Pcam2 = welch(camera_df['Cam2 Position X'], fs=60, nperseg=Nfft // 2, noverlap=Nfft // 4, nfft=Nfft)
f_cam3, Pcam3 = welch(camera_df['Cam3 Position X'], fs=60, nperseg=Nfft // 2, noverlap=Nfft // 4, nfft=Nfft)

# Frequency-Domain Plot
plt.figure(2)
plt.semilogy(f_ldv, Pldv, label='LDV Data')
plt.semilogy(f_cam1, Pcam1, label='Cam1 Data')
plt.semilogy(f_cam2, Pcam2, label='Cam2 Data')
plt.semilogy(f_cam3, Pcam3, label='Cam3 Data')
plt.title('Frequency Spectra of LDV and Camera Data')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Power Spectral Density')
plt.legend()
plt.grid(which='both', linestyle='--')
plt.savefig('Frequency_Spectra_Comparison_Three_Cameras.png')
plt.show()
