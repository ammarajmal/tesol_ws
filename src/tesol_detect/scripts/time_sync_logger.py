#!/usr/bin/env python3

import rospy
import csv
import os
import datetime
import rosgraph
import message_filters
from fiducial_msgs.msg import FiducialTransformArray

class CameraFiducialLogger:
    def __init__(self):
        # Time to stop logging
        self.time_stop = rospy.get_param('~time_stop', 10)  # Default: 2 minutes
        rospy.init_node('camera_fiducial_logger', anonymous=False)

        # Expected fiducial detection topics
        self.camera_topics = {
            1: '/sony_cam1/aruco_detect_node/fiducial_transforms',
            2: '/sony_cam2/aruco_detect_node/fiducial_transforms',
            3: '/sony_cam3/aruco_detect_node/fiducial_transforms'
        }

        # Check for active cameras
        self.active_cameras = self.get_active_cameras()
        rospy.loginfo(f"Active Cameras Detected: {self.active_cameras}")

        # Initialize storage for raw and synchronized data
        self.raw_data = {cam: [] for cam in self.active_cameras}
        self.sync_data = {cam: [] for cam in self.active_cameras}
        self.frame_count = {cam: 0 for cam in self.active_cameras}  # Count frames per camera
        self.missed_frames = {cam: 0 for cam in self.active_cameras}  # Count dropped frames

        # Prepare CSV file paths
        self.timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.base_dir = os.path.expanduser("~/Desktop/TESolution")
        os.makedirs(self.base_dir, exist_ok=True)

        self.raw_csv_files = {cam: os.path.join(self.base_dir, f"raw_fiducial_data_cam{cam}_{self.timestamp}.csv") for cam in self.active_cameras}
        self.sync_csv_files = {cam: os.path.join(self.base_dir, f"sync_fiducial_data_cam{cam}_{self.timestamp}.csv") for cam in self.active_cameras}

        # Logging duration setup
        self.logging_duration = rospy.get_param('~logging_duration', 300)  
        rospy.loginfo(f"Logging duration set to {self.logging_duration} seconds.")
        self.logging_timer = rospy.Timer(rospy.Duration(self.logging_duration), self.shutdown) 

        # Subscribe to topics
        self.subscribe_to_cameras()

        # Setup shutdown hook
        rospy.on_shutdown(self.shutdown) 

        rospy.loginfo("Camera Fiducial Logger is running...")
        rospy.spin()

    def get_active_cameras(self):
        """ Checks which cameras are actively publishing fiducial detection topics """
        active_cams = []
        master = rosgraph.Master('/rostopic')
        try:
            published_topics = master.getPublishedTopics('/')
            available_topics = {topic[0] for topic in published_topics}

            for cam, topic in self.camera_topics.items():
                if topic in available_topics:
                    active_cams.append(cam)
        except rosgraph.MasterError as e:
            rospy.logerr(f"Failed to connect to ROS Master: {e}")
            rospy.signal_shutdown("Failed to connect to ROS Master.")

        return active_cams

    def subscribe_to_cameras(self):
        """ Subscribes to active camera topics """
        if len(self.active_cameras) == 1:
            rospy.loginfo("Single camera detected, recording without synchronization.")
            self.subscribers = [
                rospy.Subscriber(self.camera_topics[self.active_cameras[0]], FiducialTransformArray, self.record_raw_data, callback_args=self.active_cameras[0])
            ]
        elif len(self.active_cameras) >= 2:
            rospy.loginfo(f"Recording with Approximate Time Synchronizer for cameras: {self.active_cameras}")
            subs = [message_filters.Subscriber(self.camera_topics[cam], FiducialTransformArray) for cam in self.active_cameras]

            # Approximate Time Synchronizer
            self.sync = message_filters.ApproximateTimeSynchronizer(subs, queue_size=10, slop=0.01, allow_headerless=True)
            self.sync.registerCallback(self.record_synchronized_data)

            # Individual subscriptions for raw data
            self.subscribers = [rospy.Subscriber(self.camera_topics[cam], FiducialTransformArray, self.record_raw_data, callback_args=cam) for cam in self.active_cameras]
        else:
            rospy.logwarn("No active cameras detected. Logger will be idle.")

    def record_raw_data(self, msg, cam_num):
        """ Records raw data from each camera before synchronization """
        timestamp = rospy.get_time()
        ros_time = msg.header.stamp.to_sec()

        self.frame_count[cam_num] += 1 

        if not msg.transforms:
            self.missed_frames[cam_num] += 1
            rospy.logwarn(f"[Camera {cam_num}] No fiducials detected! Missed Frame #{self.missed_frames[cam_num]}")
            return

        for transform in msg.transforms:
            self.raw_data[cam_num].append([
                timestamp,
                ros_time,
                cam_num,
                transform.fiducial_id,
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ])
        
    def record_synchronized_data(self, *msgs):
        """ Records synchronized data using Approximate Time Synchronizer """
        sync_time = rospy.get_time()
        ros_times = [msg.header.stamp.to_sec() for msg in msgs]

        # Compute inter-camera time offsets
        if len(ros_times) >= 2:
            time_diffs = [ros_times[i] - ros_times[i-1] for i in range(1, len(ros_times))]
            avg_delay = sum(time_diffs) / len(time_diffs)
            rospy.loginfo(f"Avg Inter-Camera Delay: {avg_delay*1000:.3f} ms")

        for i, msg in enumerate(msgs):
            cam_num = self.active_cameras[i]
            for transform in msg.transforms:
                self.sync_data[cam_num].append([
                    sync_time,
                    ros_times[i],
                    cam_num,
                    transform.fiducial_id,
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ])

    def save_data_to_csv(self):
        """ Saves raw and synchronized data to separate CSV files for each camera """
        for cam in self.active_cameras:
            # Save raw data
            with open(self.raw_csv_files[cam], 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    "System Time (s)", "ROS Header Time (s)", "Camera Number", "Fiducial ID",
                    "Position X", "Position Y", "Position Z",
                    "Rotation X", "Rotation Y", "Rotation Z", "Rotation W"
                ])
                writer.writerows(self.raw_data[cam])
            rospy.loginfo(f"Raw data from camera {cam} saved to {self.raw_csv_files[cam]}")

            # Save synchronized data
            if self.sync_data[cam]:
                with open(self.sync_csv_files[cam], 'w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([
                        "Synchronized Time (s)", "ROS Header Time (s)", "Camera Number", "Fiducial ID",
                        "Position X", "Position Y", "Position Z",
                        "Rotation X", "Rotation Y", "Rotation Z", "Rotation W"
                    ])
                    writer.writerows(self.sync_data[cam])
                rospy.loginfo(f"Synchronized data from camera {cam} saved to {self.sync_csv_files[cam]}")
            else:
                rospy.logwarn(f"No synchronized data found for camera {cam}. No sync data file created.")

    def shutdown(self, timer_event=None): 
        """ Saves data before shutting down """
        rospy.loginfo("Shutting down camera fiducial logger...")
        self.save_data_to_csv()
        rospy.loginfo("Data saved. Exiting.")
        rospy.signal_shutdown("Logging complete")
        if timer_event is not None:
            self.logging_timer.shutdown() 

if __name__ == "__main__":
    try:
        logger = CameraFiducialLogger()
    except rospy.ROSInterruptException:
        pass
