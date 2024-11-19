import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, resample, welch

DELAY = 37  # Delay in seconds to align LDV and Camera data

# Load Camera data
camera_file_path = 'Exp1_10s_2024-11-19_23-19-50.csv'
cam = pd.read_csv(camera_file_path)

# Load LDV data
ldv_file_path = 'protocol_optoNCDT-ILD1420_2024-11-19_23-19-50.049.csv'
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
data = cam['Cam3 Position X']  # Use X-axis data for analysis
data_dc_removed = data - np.mean(data)

# Interpolate Camera data to 1 KHz
time_new = np.arange(0, time.iloc[-1], 1e-3)
datau = np.interp(time_new, time, data_dc_removed)

# Resample Camera data to 60 Hz
cam_resampled = resample(datau, 60 * len(datau) // 1000)
cam_resampled = cam_resampled[DELAY:]  # Remove initial samples
time_resampled = np.linspace(0, len(cam_resampled) / 60, len(cam_resampled))
camd = pd.DataFrame({'Time (s)': time_resampled, 'Cam3 Position X': cam_resampled * 1e3})

# Time-Domain Plot
plt.figure(1)
plt.plot(resampled_time, ldvd, label='LDV Data')
plt.plot(time_resampled, camd['Cam3 Position X'].to_numpy(), label='Camera Data (X-axis)')
plt.title('LDV and Camera Data Comparison (Time Domain)')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude (mm)')
plt.legend()
plt.grid()
plt.savefig('Time_Domain_Comparison.png')  # Save plot as PNG
# plt.show()  # Display without blocking

# Frequency-Domain Analysis
fft_levels = 10
Nfft = 2 ** fft_levels

# Welch's method for LDV
f_ldv, Pldv = welch(ldvd, fs=ldv_new_sampling_frequency, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)

# Welch's method for Camera
f_cam, Pcam = welch(camd['Cam3 Position X'], fs=60, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)

# Frequency-Domain Analysis Plot
plt.figure(2)
plt.semilogy(f_ldv, Pldv, label='LDV Data')
plt.semilogy(f_cam, Pcam, label='Camera Data')
plt.title('Frequency Spectra of LDV and Camera Data')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Power Spectral Density')
plt.legend()
plt.grid(which='both', linestyle='--')
plt.savefig('Frequency_Spectra_Comparison.png')  # Save plot as PNG
plt.show()  # Display without blocking

# # Calculate SNR for Signal and Noise
# def calculate_snr(signal_power, noise_power):
#     return 10 * np.log10(signal_power / noise_power)

# # Define signal and noise frequency ranges
# signal_bandwidth = (f_ldv > 1) & (f_ldv < 50)  # 1–50 Hz as signal
# noise_bandwidth = f_ldv >= 100  # Frequencies above 100 Hz as noise

# # SNR for LDV
# ldv_signal_power = np.sum(Pldv[signal_bandwidth])
# ldv_noise_power = np.sum(Pldv[noise_bandwidth])
# ldv_snr = calculate_snr(ldv_signal_power, ldv_noise_power)

# # SNR for Camera
# cam_signal_power = np.sum(Pcam[(f_cam > 1) & (f_cam < 50)])
# cam_noise_power = np.sum(Pcam[f_cam >= 100])
# cam_snr = calculate_snr(cam_signal_power, cam_noise_power)

# # Display SNR Results
# print("LDV SNR (dB):", ldv_snr)
# print("Camera SNR (dB):", cam_snr)
