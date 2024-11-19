import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, resample, welch

DELAY = 38  # Delay in SAMPLES
# Load Camera data
camera_file_path = 'sine_10s_2024-11-20_00-39-20.csv'
cam = pd.read_csv(camera_file_path)

# Load LDV data
ldv_file_path = 'protocol_optoNCDT-ILD1420_2024-11-20_00-39-20.835.csv'
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

# Camera Data Processing
time = cam['Time (s)'] - cam['Time (s)'].iloc[0]
data = cam['Cam2 Position X']  # Use X-axis data for analysis
data_dc_removed = data - np.mean(data)

# Interpolate Camera data to 1 KHz
time_new = np.arange(0, time.iloc[-1], 1e-3)
datau = np.interp(time_new, time, data_dc_removed)

# Resample Camera data to 60 Hz
cam_resampled = resample(datau, 60 * len(datau) // 1000)
cam_resampled = cam_resampled[DELAY:]  # Remove initial samples
time_resampled = np.linspace(0, len(cam_resampled) / 60, len(cam_resampled))
camd = pd.DataFrame({'Time (s)': time_resampled, 'Cam2 Position X': cam_resampled * 1e3})

# Time-Domain Plot
plt.figure()
plt.plot(resampled_time, ldvd, label='LDV Data')
plt.plot(time_resampled, camd['Cam2 Position X'].to_numpy(), label='Camera 2 Data (X-axis)')
plt.title('LDV and Camera 2 Data Comparison (Time Domain)')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude (mm)')
plt.legend()
plt.grid()

# Frequency-Domain Analysis
fft_levels = 10
Nfft = 2 ** fft_levels

# Welch's method for LDV
f_ldv, Pldv = welch(ldvd, fs=ldv_new_sampling_frequency, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)

# Welch's method for Camera
f_cam, Pcam = welch(camd['Cam2 Position X'], fs=60, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)

# Plot Frequency-Domain Data
plt.figure()
plt.semilogy(f_ldv, Pldv, label='LDV Data')
plt.semilogy(f_cam, Pcam, label='Camera Data')
plt.title('Frequency Spectra of LDV and Camera Data')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Power Spectral Density')
plt.legend()
plt.grid(which='both', linestyle='--')
plt.show()
