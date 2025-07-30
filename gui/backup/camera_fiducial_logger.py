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
        rospy.init_node('camera_fiducial_logger', anonymous=False)

        # ✅ Define recording duration (seconds)
        self.record_duration = rospy.get_param("~record_duration", 10)  # Default: 10 seconds
        self.start_time = rospy.get_time()

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
        self.sync_data = []

        # Prepare CSV file for saving data
        self.timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.base_dir = os.path.expanduser("~/Desktop/TESolution")
        if not os.path.exists(self.base_dir):
            os.makedirs(self.base_dir)
        self.raw_csv_file = os.path.join(self.base_dir, f"raw_fiducial_data_{self.timestamp}.csv")
        self.sync_csv_file = os.path.join(self.base_dir, f"sync_fiducial_data_{self.timestamp}.csv")

        # Subscribe to topics
        self.subscribe_to_cameras()

        rospy.Timer(rospy.Duration(self.record_duration), self.shutdown)  # Stop after duration
        rospy.loginfo(f"Recording for {self.record_duration} seconds...")
        rospy.spin()

    def get_active_cameras(self):
        """ Checks which cameras are actively publishing fiducial detection topics """
        active_cams = []
        master = rosgraph.Master('/rostopic')
        published_topics = master.getPublishedTopics('/')
        available_topics = {topic[0] for topic in published_topics}

        for cam, topic in self.camera_topics.items():
            if topic in available_topics:
                active_cams.append(cam)

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

    def record_raw_data(self, msg, cam_num):
        """ Records raw data from each camera individually before synchronization """
        if rospy.get_time() - self.start_time > self.record_duration:
            return  # Stop recording if time is up

        timestamp = rospy.get_time()  # System timestamp
        ros_time = msg.header.stamp.to_sec()  # ROS header timestamp

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
        
        rospy.loginfo(f"Raw Data Logged from Camera {cam_num} - Fiducial ID: {transform.fiducial_id}")

    def record_synchronized_data(self, *msgs):
        """ Records synchronized data using Approximate Time Synchronizer """
        if rospy.get_time() - self.start_time > self.record_duration:
            return  # Stop recording if time is up

        sync_time = rospy.get_time()
        ros_times = [msg.header.stamp.to_sec() for msg in msgs]

        for i, msg in enumerate(msgs):
            cam_num = self.active_cameras[i]
            for transform in msg.transforms:
                self.sync_data.append([
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
        
        rospy.loginfo("Synchronized Data Logged")

    def save_data_to_csv(self):
        """ Saves raw and synchronized data to CSV files """
        with open(self.raw_csv_file, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                "System Time (s)", "ROS Header Time (s)", "Camera Number", "Fiducial ID",
                "Position X", "Position Y", "Position Z",
                "Rotation X", "Rotation Y", "Rotation Z", "Rotation W"
            ])
            for cam in self.raw_data:
                writer.writerows(self.raw_data[cam])
        
        rospy.loginfo(f"Raw data saved to {self.raw_csv_file}")

        if self.sync_data:
            with open(self.sync_csv_file, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    "Synchronized Time (s)", "ROS Header Time (s)", "Camera Number", "Fiducial ID",
                    "Position X", "Position Y", "Position Z",
                    "Rotation X", "Rotation Y", "Rotation Z", "Rotation W"
                ])
                writer.writerows(self.sync_data)
            
            rospy.loginfo(f"Synchronized data saved to {self.sync_csv_file}")

    def shutdown(self, event=None):
        """ Saves data before shutting down """
        rospy.loginfo("Time elapsed! Stopping recording and saving data...")
        self.save_data_to_csv()
        rospy.signal_shutdown("Logging complete")

if __name__ == "__main__":
    try:
        logger = CameraFiducialLogger()
    except rospy.ROSInterruptException:
        pass
