#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, resample, welch

# Experiment No. 1: Shaking Table - LDV Experiment
exp = 1
camera_data_path = "data_Exp1_10s_2024-09-13_00-09-34.csv"
ldv_data_path = "protocol_optoNCDT-ILD1420_2024-09-13_00-09-35.146.csv"
val = 42


cam = pd.read_csv(camera_data_path)
ldv = pd.read_csv(ldv_data_path, skiprows=6, delimiter=';')

# LDV data
ldv.columns = ['acquisition_time', 'epoch_time_ms', 'distance_mm']  # Rename the columns
ldv['epoch_time_ms'] = pd.to_numeric(ldv['epoch_time_ms'], errors='coerce')  # Convert epoch_time_ms to numeric
ldv = ldv.dropna(subset=['epoch_time_ms', 'distance_mm'])  # Drop any rows with missing values
ldv['epoch_time'] = ldv['epoch_time_ms'] / 1000  # Convert to seconds

# Calculate the original sampling frequency
ldv_sampling_frequency = 1 / np.mean(np.diff(ldv['epoch_time']))
print("Original sampling frequency:", ldv_sampling_frequency)

# Set the new sampling frequency
ldv_new_sampling_frequency = 60

# Butterworth filter
b, a = butter(5, 20/500, btype='low')
ldvf = filtfilt(b, a, ldv['distance_mm'].ravel())  # Filter the data

# Calculate the number of samples for the new sampling frequency
num_original_samples = len(ldvf)
num_new_samples = int(num_original_samples * ldv_new_sampling_frequency / ldv_sampling_frequency)
print("Number of samples after downsampling:", num_new_samples)

# Resample the data to the new sampling frequency
ldvd = resample(ldvf, num_new_samples)

# Create a new time axis for the resampled data
resampled_time = np.linspace(0, len(ldvd) / ldv_new_sampling_frequency, num_new_samples)

# Calculate the new sampling frequency for verification
ldv_new_sampling_frequency_calculated = 1 / np.mean(np.diff(resampled_time))
print("New sampling frequency calculated from resampled data:", ldv_new_sampling_frequency_calculated)

# # Plotting (Optional)
# plt.figure(1)
# plt.plot(resampled_time, ldvd)
# plt.title(f'Filtered and Resampled LDV Data (Experiment {exp})')
# plt.xlabel('Time (s)')
# plt.ylabel('Amplitude')
# plt.grid(True, which='both', linestyle='--')
# plt.savefig(f'_Filtered_resampled_ldv_data - Exp {exp}.png')
# # plt.show()

# Cam Interpolation
# now save time axes from the camera data into the time header 'Time (s)'
time = cam['Time (s)'] - cam['Time (s)'].iloc[0]
data = cam['Cam1 Position Y']

# Interpolate the camera data to 1 KHz from the original sampling frequency of 60 Hz
time_new = np.arange(0, time.iloc[-1], 1e-3)
datau = np.zeros(len(time_new))
datau = np.interp(time_new, time, data)

# Resample the camera data to 60 Hz
cam_resampled = resample(datau, 60 * len(datau)//1000)
cam_resampled = cam_resampled[val:]

# Time vector for resampled data
time_resampled = np.linspace(0, len(cam_resampled) / 60, len(cam_resampled))
camd = pd.DataFrame({'Time (s)': time_resampled, 'Cam1 Position Y': cam_resampled*1e3})

# Trim the LDV data to match the length of the camera data
ldvd = ldvd[:len(camd)]
resampled_time = resampled_time[:len(camd)]

# Plotting the resampled camera data
# plt.figure(2)
# plt.plot(resampled_time, camd['Cam1 Position Y'])
# plt.title(f'Resampled Camera Data (Experiment {exp})')
# plt.xlabel('Time (s)')
# plt.ylabel('Amplitude')
# plt.grid(True, which='both', linestyle='--')
# plt.legend(['Camera Data'])
# plt.savefig(f'_Resampled_camera_data - Exp {exp}.png')
# # plt.show()



# Plot for the LDV and Camera data comparison
plt.figure(3)
plt.plot(resampled_time, ldvd)
plt.plot(resampled_time, camd['Cam1 Position Y'])
plt.title(f'LDV and Camera Data Comparison (Experiment {exp})')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.grid(True, which='both', linestyle='--')
plt.minorticks_on()  # Enable minor ticks to increase the grid density
plt.legend(['LDV Data', 'Camera Data'])
plt.savefig(f'_LDV_camera_data_comparison - Exp {exp}.png')
# plt.show()


# FFT 
Nfft = 2**11
f_ldv, Pldv = welch(ldvd, fs=ldv_new_sampling_frequency, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)
f_cam, Pcam = welch(camd['Cam1 Position Y'], fs=60, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)

# Plotting the FFT
plt.figure(4)
plt.semilogy(f_ldv, Pldv)
plt.semilogy(f_cam, Pcam)
plt.title(f'FFT of LDV and Camera Data (Experiment {exp})')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Power Spectral Density')
plt.grid(True, which='both', linestyle='--')
plt.legend(['LDV Data', 'Camera Data'])
plt.savefig(f'_FFT_LDV_camera_data - Exp {exp}.png')
plt.show()
