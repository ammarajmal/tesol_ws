import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, resample, welch

# Set paths for the data files
camera_file_path = 'Sweep_1_30Hz_120s_2024-11-20_00-50-00.csv'
ldv_file_path = 'protocol_optoNCDT-ILD1420_2024-11-20_00-50-02.114.csv'

# Load Camera data
cam = pd.read_csv(camera_file_path)

# Load LDV data
ldv = pd.read_csv(ldv_file_path, skiprows=6, delimiter=';')
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

# Camera Data Processing for both cameras
time = cam['Time (s)'] - cam['Time (s)'].iloc[0]
cam1_data = cam['Cam1 Position X']  # X-axis for Camera 1
cam2_data = cam['Cam2 Position X']  # X-axis for Camera 2

# Remove DC Component
cam1_dc_removed = cam1_data - np.mean(cam1_data)
cam2_dc_removed = cam2_data - np.mean(cam2_data)

# Interpolate Camera data to 1 kHz
time_new = np.arange(0, time.iloc[-1], 1e-3)
cam1_u = np.interp(time_new, time, cam1_dc_removed)
cam2_u = np.interp(time_new, time, cam2_dc_removed)

# Resample Camera data to 60 Hz
cam1_resampled = resample(cam1_u, 60 * len(cam1_u) // 1000)
cam2_resampled = resample(cam2_u, 60 * len(cam2_u) // 1000)

# Generate time arrays for resampled data
time_resampled_cam1 = np.linspace(0, len(cam1_resampled) / 60, len(cam1_resampled))
time_resampled_cam2 = np.linspace(0, len(cam2_resampled) / 60, len(cam2_resampled))

# Trim LDV data to match Camera data lengths
ldvd = ldvd[:min(len(cam1_resampled), len(cam2_resampled))]
resampled_time = resampled_time[:len(ldvd)]

# Time-Domain Plot
plt.figure()
plt.plot(resampled_time, ldvd, label='LDV Data')
plt.plot(time_resampled_cam1, cam1_resampled * 1e3, label='Cam1 Data (X-axis)')
plt.plot(time_resampled_cam2, cam2_resampled * 1e3, label='Cam2 Data (X-axis)')
plt.title('LDV and Camera Data Comparison (Time Domain)')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude (mm)')
plt.legend()
plt.grid()
plt.savefig('Time_Domain_Comparison_Cam1_Cam2_LDV.png')

# Frequency-Domain Analysis
fft_levels = 10
Nfft = 2 ** fft_levels

# Welch's method for LDV
f_ldv, Pldv = welch(ldvd, fs=ldv_new_sampling_frequency, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)

# Welch's method for Cameras
f_cam1, Pcam1 = welch(cam1_resampled, fs=60, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)
f_cam2, Pcam2 = welch(cam2_resampled, fs=60, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)

# Frequency-Domain Plot
plt.figure()
plt.semilogy(f_ldv, Pldv, label='LDV Data')
plt.semilogy(f_cam1, Pcam1, label='Cam1 Data')
plt.semilogy(f_cam2, Pcam2, label='Cam2 Data')
plt.title('Frequency Spectra of LDV and Camera Data')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Power Spectral Density')
plt.legend()
plt.grid(which='both', linestyle='--')
plt.savefig('Frequency_Spectra_Comparison_Cam1_Cam2_LDV.png')
plt.show()
