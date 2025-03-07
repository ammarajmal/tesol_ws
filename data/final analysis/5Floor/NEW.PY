# -*- coding: utf-8 -*-
#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, resample, welch, find_peaks

# Experiment No. 1: Shaking Table - LDV Experiment
exp = 1
seconds_to_display = 'full'  # Duration of data to display in time domain plots (in seconds) or 'full' to display all data
camera_data_path = "TS1_125s_2025-03-06_17-04-55.csv"
ldv_data_path = "protocol_optoNCDT-ILD1420_2025-03-06_08-04-55.633.csv"
val = 5
Nfft = 2**12

# Load camera data
cam = pd.read_csv(camera_data_path)

# Load LDV data
ldv = pd.read_csv(ldv_data_path, skiprows=6, delimiter=';')
ldv.columns = ldv.columns.str.strip()
ldv['Epoch time (ms)'] = ldv['Epoch time (ms)'].astype(float)
ldv['Distance1 (mm)'] = ldv['Distance1 (mm)'].astype(float)

# Renaming the columns for easier reference
ldv.columns = ['acquisition_time', 'epoch_time_ms', 'distance_mm']
ldv['epoch_time'] = ldv['epoch_time_ms'] / 1000  # Convert to seconds

# Calculate original sampling frequency
ldv_sampling_frequency = 1 / np.mean(np.diff(ldv['epoch_time']))

# Remove DC component from LDV data
ldv_dc_removed = ldv['distance_mm'] - np.mean(ldv['distance_mm'])

# Apply High-Pass Butterworth Filter (cutoff = 0.3 Hz)
def highpass_filter(data, cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return filtfilt(b, a, data)

ldv_filtered = highpass_filter(ldv_dc_removed, 0.3, ldv_sampling_frequency)

# Apply Low-Pass Filter (Existing Butterworth 20Hz cutoff)
b, a = butter(5, 20/500, btype='low')
ldvf = filtfilt(b, a, ldv_filtered.ravel())

# Resample LDV data
ldv_new_sampling_frequency = 60
num_new_samples = int(len(ldvf) * ldv_new_sampling_frequency / ldv_sampling_frequency)
ldvd = resample(ldvf, num_new_samples)
resampled_time = np.linspace(0, len(ldvd) / ldv_new_sampling_frequency, num_new_samples)

# Camera Interpolation and Resampling
time = cam['Cam3 System Time (s)'] - cam['Cam3 System Time (s)'].iloc[0]
data = cam['Cam3 Position X']

# Remove DC component from Camera data
data_dc_removed = data - np.mean(data)

# Apply High-Pass Filter to Camera Data
camera_sampling_frequency = 1 / np.mean(np.diff(time))
data_filtered = highpass_filter(data_dc_removed, 0.3, camera_sampling_frequency)

# Interpolate camera data to 1 KHz
time_new = np.arange(0, time.iloc[-1], 1e-3)
datau = np.interp(time_new, time, data_filtered)

# Resample the camera data to 60 Hz
cam_resampled = resample(datau, 60 * len(datau) // 1000)[val:]
time_resampled = np.linspace(0, len(cam_resampled) / 60, len(cam_resampled))
camd = pd.DataFrame({'Time (s)': time_resampled, 'Cam3 Position X': cam_resampled * 1e3})

# Trim LDV data to match camera data length
ldvd = ldvd[:len(camd)]
resampled_time = resampled_time[:len(camd)]

# Compute Welch's PSD using Hanning Window
nperseg = min(Nfft // 2, len(ldvd))
overlap = min(Nfft // 4, nperseg - 1)
window = np.hanning(nperseg)

f_ldv, Pldv = welch(ldvd, fs=ldv_new_sampling_frequency, window=window, nperseg=nperseg, noverlap=overlap, nfft=Nfft)
f_cam, Pcam = welch(camd['Cam3 Position X'], fs=60, window=window, nperseg=nperseg, noverlap=overlap, nfft=Nfft)

# Find peaks in the power spectral density
peaks_ldv, _ = find_peaks(Pldv, height=np.max(Pldv) * 0.1)  # Detect peaks with at least 10% of max height
peaks_cam, _ = find_peaks(Pcam, height=np.max(Pcam) * 0.1)

# Sort the peaks by prominence and get the top 5 modes
top_peaks_ldv = peaks_ldv[np.argsort(Pldv[peaks_ldv])][-5:][::-1]  # Get 5 highest peaks
top_peaks_cam = peaks_cam[np.argsort(Pcam[peaks_cam])][-5:][::-1]

# Extract the corresponding frequency values
modes_ldv = f_ldv[top_peaks_ldv]
modes_cam = f_cam[top_peaks_cam]

print("Top 5 Mode Frequencies from LDV:", modes_ldv)
print("Top 5 Mode Frequencies from Camera:", modes_cam)

# Plot FFT with Peaks
plt.figure(1)
plt.semilogy(f_ldv, Pldv, label='LDV Data')
plt.semilogy(f_cam, Pcam, label='Camera Data')
plt.scatter(modes_ldv, Pldv[top_peaks_ldv], color='red', label='LDV Modes', marker='o')
plt.scatter(modes_cam, Pcam[top_peaks_cam], color='blue', label='Camera Modes', marker='x')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Power Spectral Density')
plt.title('FFT with Mode Peaks')
plt.grid(True, linestyle='--')
plt.legend()
plt.savefig('_FFT_LDV_camera_data_modes.png')
plt.show()
