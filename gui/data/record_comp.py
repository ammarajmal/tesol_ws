#!/usr/bin/env python3
import rospy
import signal
from fiducial_msgs.msg import FiducialTransformArray
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime
import time
import matplotlib.ticker as ticker
def signal_handler(sig, frame):
    print('Exiting...')
    rospy.signal_shutdown('Exiting...')
    plt.close('all')
    exit(0)

class DataCollector:
    def __init__(self):
        rospy.init_node('data_collector')
        self.camera_name = 'sony_cam2'  # Replace with your camera name
        self.marker_data = {}
        self.recording_time = 20  # Recording time in seconds
        self.start_time = None
        self.end_time = None
        self.all_timestamps = []  # Store all timestamps

        self.fid_sub = rospy.Subscriber(f"/{self.camera_name}/aruco_detect_node/fiducial_transforms", FiducialTransformArray, self.fid_callback)
        self.cam_sub = rospy.Subscriber(f"/{self.camera_name}/aruco_detect_node/fiducial_transforms_camera", FiducialTransformArray, self.cam_callback)

        signal.signal(signal.SIGINT, signal_handler)  # Capture CTRL+C

        rospy.spin()

    def fid_callback(self, msg):
        if self.start_time is None:
            self.start_time = msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9  # Record start time from message header
        elapsed_time = time.time() - self.start_time
        if elapsed_time <= self.recording_time:
            for transform in msg.transforms:
                fid_id = transform.fiducial_id
                timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9  # Extract message timestamp
                self.all_timestamps.append(timestamp)
                if fid_id not in self.marker_data:
                    self.marker_data[fid_id] = {'marker': [], 'camera': [], 'timestamps': []}
                self.marker_data[fid_id]['marker'].append([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
                self.marker_data[fid_id]['timestamps'].append(timestamp)

    def cam_callback(self, msg):
        if self.start_time is None:
            self.start_time = msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9  # Record start time from message header
        elapsed_time = time.time() - self.start_time
        if elapsed_time <= self.recording_time:
            for transform in msg.transforms:
                fid_id = transform.fiducial_id
                timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9  # Extract message timestamp
                self.all_timestamps.append(timestamp)
                if fid_id in self.marker_data:
                    self.marker_data[fid_id]['camera'].append([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
                    self.marker_data[fid_id]['timestamps'].append(timestamp)

        if elapsed_time >= self.recording_time and not self.end_time:
            self.end_time = time.time()
            self.plot_data()

    import matplotlib.ticker as ticker

    def plot_data(self):
        for fid_id, data in self.marker_data.items():
            marker_x, marker_y, marker_z = zip(*data['marker'])
            camera_x, camera_y, camera_z = zip(*data['camera'])
            timestamps = data['timestamps']

            # Convert timestamps to elapsed time in seconds from the start
            start_time = timestamps[0]
            elapsed_time = [ts - start_time for ts in timestamps]

            # Ensure data and elapsed_time have the same length
            min_len = min(len(marker_x), len(camera_x), len(elapsed_time))
            marker_x = marker_x[:min_len]
            marker_y = marker_y[:min_len]
            marker_z = marker_z[:min_len]
            camera_x = camera_x[:min_len]
            camera_y = camera_y[:min_len]
            camera_z = camera_z[:min_len]
            elapsed_time = elapsed_time[:min_len]

            # Calculate displacement (assuming first point as reference) and convert to millimeters
            displacement_marker_x = [(x - marker_x[0]) * 1000 for x in marker_x]
            displacement_marker_y = [(y - marker_y[0]) * 1000 for y in marker_y]
            displacement_marker_z = [(z - marker_z[0]) * 1000 for z in marker_z]

            displacement_camera_x = [(x - camera_x[0]) * 1000 for x in camera_x]
            displacement_camera_y = [(y - camera_y[0]) * 1000 for y in camera_y]
            displacement_camera_z = [(z - camera_z[0]) * 1000 for z in camera_z]

            fig, axs = plt.subplots(3, 1, figsize=(10, 12))

            # Format y-axis ticks to 4 decimal places
            formatter = ticker.FormatStrFormatter('%.4f')

            # Plot displacement with elapsed_time
            axs[0].plot(elapsed_time, displacement_marker_x, label='Marker X (mm)')
            axs[0].plot(elapsed_time, displacement_camera_x, label='Camera X (mm)')
            axs[0].set_ylabel('X Displacement (mm)')
            axs[0].legend()
            axs[0].set_xlabel('Time (s)')
            axs[0].yaxis.set_major_formatter(formatter)

            axs[1].plot(elapsed_time, displacement_marker_y, label='Marker Y (mm)')
            axs[1].plot(elapsed_time, displacement_camera_y, label='Camera Y (mm)')
            axs[1].set_ylabel('Y Displacement (mm)')
            axs[1].legend()
            axs[1].set_xlabel('Time (s)')
            axs[1].yaxis.set_major_formatter(formatter)

            axs[2].plot(elapsed_time, displacement_marker_z, label='Marker Z (mm)')
            axs[2].plot(elapsed_time, displacement_camera_z, label='Camera Z (mm)')
            axs[2].set_ylabel('Z Displacement (mm)')
            axs[2].legend()
            axs[2].set_xlabel('Time (s)')
            axs[2].yaxis.set_major_formatter(formatter)

            plt.title(f'Fiducial ID: {fid_id}')
            plt.show()



if __name__ == '__main__':
    try:
        data_collector = DataCollector()
    except rospy.ROSInterruptException:
        pass