#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, resample, welch

# Experiment No. 1: Shaking Table - LDV Experiment
exp = 9
camera_data_path = "data_Exp1_10s_2024-09-20_02-55-38.csv"
ldv_data_path = "protocol_optoNCDT-ILD1420_2024-09-20_02-55-38.476.csv"
val = 41

# Load data
cam = pd.read_csv(camera_data_path)
ldv = pd.read_csv(ldv_data_path, skiprows=6, delimiter='\t')
ldv.columns = ldv.columns.str.strip()

# LDV data processing

# Replacing commas with dots for numerical data
ldv['Epoch time (ms)'] = ldv['Epoch time (ms)'].str.replace(',', '.').astype(float)
ldv['Distance1 (mm)'] = ldv['Distance1 (mm)'].str.replace(',', '.').astype(float)

# Renaming the columns for easier reference
ldv.columns = ['acquisition_time', 'epoch_time_ms', 'distance_mm']
ldv['epoch_time'] = ldv['epoch_time_ms'] / 1000  # Convert to seconds

# Calculate original sampling frequency
ldv_sampling_frequency = 1 / np.mean(np.diff(ldv['epoch_time']))

# Remove DC component from LDV data
ldv_dc_removed = ldv['distance_mm'] - np.mean(ldv['distance_mm'])

# Set new sampling frequency
ldv_new_sampling_frequency = 60

# Butterworth filter
b, a = butter(5, 20/500, btype='low')
ldvf = filtfilt(b, a, ldv_dc_removed.ravel())  # Filter data

# Resample data
num_original_samples = len(ldvf)
num_new_samples = int(num_original_samples * ldv_new_sampling_frequency / ldv_sampling_frequency)
ldvd = resample(ldvf, num_new_samples)
resampled_time = np.linspace(0, len(ldvd) / ldv_new_sampling_frequency, num_new_samples)

# Camera data interpolation and resampling
time = cam['Time (s)'] - cam['Time (s)'].iloc[0]
data = cam[['Cam1 Position Y', 'Cam2 Position Y', 'Cam3 Position Y']].values  # Added third camera column
# Remove DC Component from Camera data
data_dc_removed = data - np.mean(data, axis=0)
# Interpolate data to 1 KHz
time_new = np.arange(0, time.iloc[-1], 1e-3)
datau = np.zeros((len(time_new), 3))  # Updated for three cameras
for i in range(3):  # Loop updated for three cameras
    datau[:, i] = np.interp(time_new, time, data_dc_removed[:, i])

# Resample camera data to 60 Hz
cam_resampled = resample(datau, 60 * len(datau)//1000)
cam_resampled = cam_resampled[val:]
time_resampled = np.linspace(0, len(cam_resampled) / 60, len(cam_resampled))
camd = pd.DataFrame({
    'Time (s)': time_resampled, 
    'Cam1 Position Y': cam_resampled[:, 0]*1e3, 
    'Cam2 Position Y': cam_resampled[:, 1]*1e3, 
    'Cam3 Position Y': cam_resampled[:, 2]*1e3  # Added third camera
})

# Trim LDV data to match camera data length
ldvd = ldvd[:len(camd)]
resampled_time = resampled_time[:len(camd)]

# Find max and min values for LDV and Camera data
max_ldv = ldvd.max()
min_ldv = ldvd.min()
max_cam1 = camd['Cam1 Position Y'].max()
min_cam1 = camd['Cam1 Position Y'].min()
max_cam2 = camd['Cam2 Position Y'].max()
min_cam2 = camd['Cam2 Position Y'].min()
max_cam3 = camd['Cam3 Position Y'].max()  # Added max/min for Cam3
min_cam3 = camd['Cam3 Position Y'].min()

# Plot for LDV and Camera data comparison
plt.figure(1)
plt.plot(resampled_time, ldvd, label=f'LDV (Max: {max_ldv:.2f}, Min: {min_ldv:.2f})')
plt.plot(resampled_time, camd['Cam1 Position Y'], label=f'Cam1 (Max: {max_cam1:.2f}, Min: {min_cam1:.2f})')
plt.plot(resampled_time, camd['Cam2 Position Y'], label=f'Cam2 (Max: {max_cam2:.2f}, Min: {min_cam2:.2f})')
plt.plot(resampled_time, camd['Cam3 Position Y'], label=f'Cam3 (Max: {max_cam3:.2f}, Min: {min_cam3:.2f})')  # Added Cam3 plot
plt.title(f'LDV and Camera Data Comparison')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.grid(True, which='both', linestyle='--')
plt.minorticks_on()
plt.legend(loc='lower left')
plt.text(0.05, 0.96, f'Difference in Max - Cam1: {abs(max_ldv - max_cam1):.2f}, Min: {abs(min_ldv - min_cam1):.2f}', transform=plt.gca().transAxes, color='red')
plt.text(0.05, 0.93, f'Difference in Max - Cam2: {abs(max_ldv - max_cam2):.2f}, Min: {abs(min_ldv - min_cam2):.2f}', transform=plt.gca().transAxes, color='blue')
plt.text(0.05, 0.90, f'Difference in Max - Cam3: {abs(max_ldv - max_cam3):.2f}, Min: {abs(min_ldv - min_cam3):.2f}', transform=plt.gca().transAxes, color='green')  # Added text for Cam3
plt.savefig(f'_LDV_camera_data_comparison - Exp {exp}.png')

# FFT
Nfft = 2**11
f_ldv, Pldv = welch(ldvd, fs=ldv_new_sampling_frequency, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)
f_cam1, Pcam1 = welch(camd['Cam1 Position Y'], fs=60, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)
f_cam2, Pcam2 = welch(camd['Cam2 Position Y'], fs=60, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)
f_cam3, Pcam3 = welch(camd['Cam3 Position Y'], fs=60, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)  # Added FFT for Cam3

# Find max frequencies
max_freq_ldv = f_ldv[np.argmax(Pldv)]
max_freq_cam1 = f_cam1[np.argmax(Pcam1)]
max_freq_cam3 = f_cam3[np.argmax(Pcam3)]  # Added max freq for Cam3

# Plot FFT
plt.figure(2)
plt.semilogy(f_ldv, Pldv, label=f'LDV Data (Max Freq: {max_freq_ldv:.2f} Hz)')
plt.semilogy(f_cam1, Pcam1, label=f'Cam1 Data (Max Freq: {max_freq_cam1:.2f} Hz)')
plt.semilogy(f_cam2, Pcam2, label=f'Cam2 Data (Max Freq: {f_cam2[np.argmax(Pcam2)]:.2f} Hz)')
plt.semilogy(f_cam3, Pcam3, label=f'Cam3 Data (Max Freq: {max_freq_cam3:.2f} Hz)')  # Added Cam3 plot
plt.title(f'FFT of LDV and Camera Data (Experiment {exp})')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Power Spectral Density')
plt.grid(True, which='both', linestyle='--')
plt.legend(loc='lower left')
plt.savefig(f'_FFT_LDV_camera_data - Exp {exp}.png')
plt.show()

