#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import datetime
import tkinter as tk
import customtkinter as ctk
import pandas as pd
# import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
# import subprocess
import csv
import message_filters
from fiducial_msgs.msg import FiducialTransformArray
# import re
# import psutil
import roslaunch.rlutil
import rospy
import rospkg
import roslaunch
# from fast_cam.msg import CameraSpecs
themes = {'blue': ("#3B8ED0", "#1F6AA5", "#1f82d1"),
          'green': ("#2CC985", "#2FA572"),
          'dark-blue': ("#3a7ebf", "#1f538d"),
          'red': ("#fa5f5a", "#ba3732")
          }
# select value of COLOR_SELECT from (0: blue, 1: green, 2: dark-blue)
COLOR_SELECT = list(themes.keys())[0]
# Modes: "System" (standard), "Dark", "Light"
ctk.set_appearance_mode("System")
# Themes: "blue" (standard), "green", "dark-blue"
ctk.set_default_color_theme(COLOR_SELECT)

class NodeGUI(ctk.CTk):
    def __init__(self, *args, **kwargs):
        super(NodeGUI, self).__init__(*args, **kwargs)
        self.title("Displacement Measurement System")
        self.geometry("1200x580")
        self.resizable(1, 1)
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.sub1 = None
        self.sub2 = None
        self.sub3 = None
        self.ats = None
        self.cam_first = None
        self.cam_second = None
        self.cam_third = None
        self.cam1_status = False
        self.cam2_status = False
        self.cam3_status = False
        self.detect1_status = False
        self.detect2_status = False
        self.detect3_status = False

        self.experiment_name = 'Exp1'
        self.file_name = None
        self.experiment_dur = 10 # seconds
        self.exp_name_var = tk.StringVar(self, self.experiment_name)
        self.exp_dur_var = tk.StringVar(self, self.experiment_dur)
    
        # self.image_width = '640'
        # self.image_height = '480'
        self.image_height = '1080'
        self.image_width = '1920'
        
        
        self.camera_resolution = self.image_width + 'x' + self.image_height
        self.camera_fps = '60'

        self.update_interval = 1000 # ms
        self.detection_rate_timeout = 5 # timeout for detection rate calculation

        self.is_detection_active = False
        self.is_data_collection_active = False
        self.collectected_data_world = []
        self.collectected_data_camera = []
        self.collectected_data_displacement = []


        self.board_size = '6x9' # default board size for calibration
        self.square_size = '0.0064' # default square size for calibration in meters = 6.4mm
        self.sq_size_var = tk.StringVar(self, self.square_size)
        self.board_size_var = tk.StringVar(self, self.board_size)

        self.marker_dim = '0.020' # ARUCO marker dimension in meters
        self.marker_dict = "DICT_7X7_1000" #  ARUCO marker dictionary (DICT_7X7_1000)
        self.marker_dim_var = tk.StringVar(self, self.marker_dim)
        self.marker_dict_var = tk.StringVar(self, self.marker_dict)

        self.cam_pkg = 'sony_cam'
        self.detect_pkg = 'tesol_detect'
        try:
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            self.cam_launch_path = rospkg.RosPack().get_path(self.cam_pkg) + '/launch/'
            self.detect_launch_path = rospkg.RosPack().get_path(self.detect_pkg) + '/launch/'
            self.cam_launch_file = f'{self.cam_launch_path}use_cam.launch'
            self.cam_view_launch_file = f'{self.cam_launch_path}use_viewcam.launch'
            self.cam_calib_launch_file = f'{self.cam_launch_path}calib.launch'
            self.detect_launch_file = f'{self.detect_launch_path}use_tesol.launch'
        except rospkg.common.ResourceNotFound:
            print('ROS packages not found')
            self.cam_launch_file = None
            self.cam_view_launch_file = None
            self.cam_calib_launch_file = None
            self.detect_launch_file = None
        
        
        self.label_height = 0.055
        self.ver_space = 0.06
        self.frame_width = 1 - (self.ver_space*4)
        self.frame_height = 0.2725

        # process management
        self.running_processes = {}

        # gui widgets initialization
        self.create_widgets()
        # self.update()
        self.left_frame = None
        
        

    def create_widgets(self)-> None:
        ''' Starts the GUI widgets creation '''

        self.create_left_frame()
        self.create_middle_first_frame()
        self.create_middle_second_frame()
        self.create_right_frame()
    def create_right_frame(self)-> None:
        ''' Creates the right frame for displaying camera images '''
        self.right_frame = tk.Frame(self, bg=themes[COLOR_SELECT][2])
        self.right_frame.place(relx=0.75, rely=0, relwidth=0.25, relheight=1)
        self.create_right_top_frame()
        self.create_right_bottom_frame()
    def create_right_top_frame(self)-> None:
        ''' Creates the top frame in the right frame '''
        self.right_top_frame = ctk.CTkFrame(self.right_frame,fg_color=themes[COLOR_SELECT][0])
        self.right_top_frame.place(relx=0.5, rely=self.ver_space, relwidth=self.frame_width, relheight=self.label_height, anchor='n')
        self.create_right_top_frame_widgets()
    def create_right_top_frame_widgets(self)-> None:
        ''' Creates the widgets in the top frame in the right frame '''
        self.right_top_frame_label = ctk.CTkLabel(self.right_top_frame, text='SYSTEM INFORMATION', text_color='white')
        self.right_top_frame_label.place(relx=0.5, rely=0.5, anchor='center')
    def create_right_bottom_frame(self)-> None:
        ''' Creates the bottom frame in the right frame '''
        self.right_bottom_frame = ctk.CTkFrame(self.right_frame, fg_color=themes[COLOR_SELECT][0])
        self.right_bottom_frame.place(relx=0.5, rely=(self.ver_space*2)+self.label_height, relwidth=self.frame_width, relheight=1-(self.ver_space*3)-self.label_height, anchor='n')
        self.create_right_bottom_frame_widgets()
    def create_right_bottom_frame_widgets(self)-> None:
        ''' creates button for camera 1 saving data and plotting data'''
        self.cam_custom_record_button = ctk.CTkButton(self.right_bottom_frame, text='cam 2 comparison', command=lambda:self.custom_cam_record(2))
        self.cam_custom_plot_button = ctk.CTkButton(self.right_bottom_frame, text='cam 2 plot', command=self.plot_data_custom)
        self.cam_custom_record_button.place(relx=0.5, rely=0.1, anchor='n')
        self.cam_custom_plot_button.place(relx=0.5, rely=0.3, anchor='n')
    def custom_cam_record(self, cam_num=1):
        ''' Records the data '''
        rospy.loginfo(f'Recording Pose Data for Camera {cam_num}')
        self.is_data_collection_active = True
        self.cam_first = cam_num
        self.record_data_param_update()
        print(f'File Name: {self.file_name}')

        
        self.sub_world = rospy.Subscriber(f'/sony_cam{cam_num}_detect/world_fiducial_transforms', FiducialTransformArray, self.record_world)
        self.sub_camera = rospy.Subscriber(f'/sony_cam{cam_num}_detect/camera_fiducial_transforms', FiducialTransformArray, self.record_camera)
        self.sub_displacement = rospy.Subscriber(f'/sony_cam{cam_num}_detect/fiducial_transforms', FiducialTransformArray, self.record_displacement)
        
        rospy.Timer(rospy.Duration(self.experiment_dur), self.stop_data_collection_custom, oneshot=True)
        
    def record_world(self, msg):
        if not self.is_data_collection_active:
            return
        timestamp = rospy.get_time()
        for transform in msg.transforms:
            print(f'Fiducial ID: {transform.fiducial_id}, Position: ({transform.transform.translation.x}, {transform.transform.translation.y}, {transform.transform.translation.z}), Rotation: ({transform.transform.rotation.x}, {transform.transform.rotation.y}, {transform.transform.rotation.z}, {transform.transform.rotation.w})')
            self.collectected_data_world.append([timestamp, transform.fiducial_id, transform.transform.translation.x,
                                                 transform.transform.translation.y, transform.transform.translation.z,
                                                 transform.transform.rotation.x, transform.transform.rotation.y,
                                                 transform.transform.rotation.z, transform.transform.rotation.w])
    
    def record_camera(self, msg):
        if not self.is_data_collection_active:
            return
        timestamp = rospy.get_time()
        for transform in msg.transforms:
            self.collectected_data_camera.append([timestamp, transform.fiducial_id, transform.transform.translation.x,
                                                  transform.transform.translation.y, transform.transform.translation.z,
                                                  transform.transform.rotation.x, transform.transform.rotation.y,
                                                  transform.transform.rotation.z, transform.transform.rotation.w])
    
    def record_displacement(self, msg):
        if not self.is_data_collection_active:
            return
        timestamp = rospy.get_time()
        for transform in msg.transforms:
            self.collectected_data_displacement.append([timestamp, transform.fiducial_id, transform.transform.translation.x,
                                                        transform.transform.translation.y, transform.transform.translation.z,
                                                        transform.transform.rotation.x, transform.transform.rotation.y,
                                                        transform.transform.rotation.z, transform.transform.rotation.w])
    
    def stop_data_collection_custom(self, event):
        self.is_data_collection_active = False
        self.sub_world.unregister()
        self.sub_camera.unregister()
        self.sub_displacement.unregister()
        self.save_to_csv_custom()
    def save_to_csv_custom(self):
        ''' Saves the data to a CSV file '''
        with open(self.file_name, 'w', newline='') as file:
            writer = csv.writer(file)
            # Write the header
            writer.writerow(['Time (s)',
                            f'Cam{self.cam_first} Fiducial ID',
                            'Camera Position X', 'Camera Position Y', 'Camera Position Z',
                            'Camera Rotation W', 'Camera Rotation X', 'Camera Rotation Y', 'Camera Rotation Z',
                            'World Position X', 'World Position Y', 'World Position Z',
                            'World Rotation W', 'World Rotation X', 'World Rotation Y', 'World Rotation Z',
                            'Displacement Position X', 'Displacement Position Y', 'Displacement Position Z',
                            'Displacement Rotation W', 'Displacement Rotation X', 'Displacement Rotation Y', 'Displacement Rotation Z'])

            # Ensure all lists have the same length by filling missing values with None
            max_length = max(len(self.collectected_data_world), len(self.collectected_data_camera), len(self.collectected_data_displacement))

            for i in range(max_length):
                world_data = self.collectected_data_world[i] if i < len(self.collectected_data_world) else [None] * 9
                camera_data = self.collectected_data_camera[i] if i < len(self.collectected_data_camera) else [None] * 9
                displacement_data = self.collectected_data_displacement[i] if i < len(self.collectected_data_displacement) else [None] * 9

                # Assuming all data points share the same timestamp and fiducial ID, otherwise, adjust accordingly
                timestamp = world_data[0] if world_data[0] is not None else camera_data[0] if camera_data[0] is not None else displacement_data[0]
                fiducial_id = world_data[1] if world_data[1] is not None else camera_data[1] if camera_data[1] is not None else displacement_data[1]

                row = [timestamp, fiducial_id] + \
                    camera_data[2:] + \
                    world_data[2:] + \
                    displacement_data[2:]
                
                writer.writerow(row)

        rospy.loginfo(f'Data saved to {self.file_name}')

        


    def plot_data_custom(self):
        ''' Plots the data '''
        print('Experiment name:', self.experiment_name)
        print('File name:', self.file_name)
        print('Experiment duration:', self.experiment_dur)
        print('Plotting Data')
        data = pd.read_csv(self.file_name)
        
        # Convert Unix timestamp to seconds relative to the start of the experiment
        start_time = data['Time (s)'].iloc[0]
        data['Time (s)'] = (data['Time (s)'] - start_time)
        
        # Plot Camera Positions
        fig1, axs1 = plt.subplots(3, 1, figsize=(15, 15))
        for i, axis in enumerate(['X', 'Y', 'Z']):
            col_name = f'Camera Position {axis}'
            if col_name in data.columns:
                axs1[i].plot(data['Time (s)'].to_numpy(), data[col_name].to_numpy(), label=f'Camera Position {axis}')
                axs1[i].set_title(f'Camera Position {axis} - {self.experiment_name}')
                axs1[i].set_xlabel('Time (s)')
                axs1[i].set_ylabel('Position (m)')
        
        # Plot World Positions
        fig2, axs2 = plt.subplots(3, 1, figsize=(15, 15))
        for i, axis in enumerate(['X', 'Y', 'Z']):
            col_name = f'World Position {axis}'
            if col_name in data.columns:
                axs2[i].plot(data['Time (s)'].to_numpy(), data[col_name].to_numpy(), label=f'World Position {axis}')
                axs2[i].set_title(f'World Position {axis} - {self.experiment_name}')
                axs2[i].set_xlabel('Time (s)')
                axs2[i].set_ylabel('Position (m)')
                axs2[i].legend()
                axs2[i].grid(True)
        fig2.savefig(self.file_name.replace('.csv', '_world_positions.png'))
        
        # Plot Displacement Positions
        fig3, axs3 = plt.subplots(3, 1, figsize=(15, 15))
        for i, axis in enumerate(['X', 'Y', 'Z']):
            col_name = f'Displacement Position {axis}'
            if col_name in data.columns:
                axs3[i].plot(data['Time (s)'].to_numpy(), data[col_name].to_numpy(), label=f'Displacement Position {axis}')
                axs3[i].set_title(f'Displacement Position {axis} - {self.experiment_name}')
                axs3[i].set_xlabel('Time (s)')
                axs3[i].set_ylabel('Position (m)')
                axs3[i].legend()
                axs3[i].grid(True)
        fig3.savefig(self.file_name.replace('.csv', '_displacement_positions.png'))
        
        # Plot Comparison of Positions
        fig4, axs4 = plt.subplots(3, 1, figsize=(15, 15))
        for i, axis in enumerate(['X', 'Y', 'Z']):
            for frame in ['Camera', 'World', 'Displacement']:
                col_name = f'{frame} Position {axis}'
                if col_name in data.columns:
                    axs4[i].plot(data['Time (s)'].to_numpy(), data[col_name].to_numpy(), label=f'{frame} Position {axis}')
            axs4[i].set_title(f'Comparison of {axis} Positions - {self.experiment_name}')
            axs4[i].set_xlabel('Time (s)')
            axs4[i].set_ylabel('Position (m)')
            axs4[i].legend()
            axs4[i].grid(True)
        fig4.savefig(self.file_name.replace('.csv', '_comparison_positions.png'))
        
        plt.show()
        rospy.loginfo(f'Plots saved to {self.file_name.replace(".csv", "_*.png")}')


    def create_middle_second_frame(self)-> None:
        ''' Creates the middle frame for setting system parameters '''
        self.middle_second_frame = tk.Frame(self, bg=themes[COLOR_SELECT][0])
        self.middle_second_frame.place(relx=0.5, rely=0, relwidth=0.25, relheight=1)
        self.create_middle_second_top_frame()
        self.create_middle_second_center_frame()
        self.create_middle_second_bottom_frame()
    def create_middle_second_top_frame(self)-> None:
        ''' Creates the top frame in the middle second frame '''
        self.middle_second_top_frame = ctk.CTkFrame(self.middle_second_frame)
        self.middle_second_top_frame.place(relx=0.5, rely=self.ver_space, relwidth=self.frame_width, relheight=self.label_height, anchor='n')
        self.create_middle_second_top_frame_widgets()
    def create_middle_second_top_frame_widgets(self)-> None:
        ''' Creates the widgets in the top frame in the middle second frame '''
        self.middle_second_top_frame_label = ctk.CTkLabel(self.middle_second_top_frame, text='RECORD & PLOT DATA')
        self.middle_second_top_frame_label.place(relx=0.5, rely=0.5, anchor='center')
    def create_middle_second_center_frame(self)-> None:
        ''' Creates the center frame in the middle second frame '''
        self.middle_second_center_frame = ctk.CTkFrame(self.middle_second_frame)
        self.middle_second_center_frame.place(relx=0.5, rely=(self.ver_space*2)+self.label_height, relwidth=self.frame_width, relheight=self.frame_height+0.1, anchor='n')
        self.create_middle_second_center_frame_widgets()
    def create_middle_second_center_frame_widgets(self)-> None:
        ''' Creates the widgets in the center frame in the middle second frame '''
        self.middle_second_center_record_label = ctk.CTkLabel(self.middle_second_center_frame, text='RECORD DATA')
        self.middle_second_center_record_label.place(relx=0.5, rely=0.05, anchor='n')
        self.middle_second_center_exp_label = ctk.CTkLabel(self.middle_second_center_frame, text='Experiment Name: ')
        self.middle_second_center_exp_label.place(relx=0.09, rely=0.2)
        self.middle_second_center_exp_entry = ctk.CTkEntry(self.middle_second_center_frame, textvariable=self.exp_name_var)
        self.middle_second_center_exp_entry.place(relx=0.75, rely=0.2, anchor='n', relwidth=0.3)
        self.middle_second_center_dur_label = ctk.CTkLabel(self.middle_second_center_frame, text='Experiment Duration(s):')
        self.middle_second_center_dur_label.place(relx=0.09, rely=0.37)
        self.middle_second_center_dur_entry = ctk.CTkEntry(self.middle_second_center_frame, textvariable=self.exp_dur_var)
        self.middle_second_center_dur_entry.place(relx=0.8, rely=0.37, anchor='n', relwidth=0.2)
        self.middle_second_center_rec1_button = ctk.CTkButton(self.middle_second_center_frame, text='1', command=lambda:self.record_data(1))
        self.middle_second_center_rec1_button.place(relx=0.1, rely=0.58, relwidth=0.2)
        self.middle_second_center_rec2_button = ctk.CTkButton(self.middle_second_center_frame, text='2', command=lambda:self.record_data(2))
        self.middle_second_center_rec2_button.place(relx=0.4, rely=0.58, relwidth=0.2)
        self.middle_second_center_rec3_button = ctk.CTkButton(self.middle_second_center_frame, text='3', command=lambda:self.record_data(3))
        self.middle_second_center_rec3_button.place(relx=0.7, rely=0.58, relwidth=0.2)
        self.middle_second_center_recall_button = ctk.CTkButton(self.middle_second_center_frame, text='RECORD ALL', command=self.recall_data)
        self.middle_second_center_recall_button.place(relx=0.5, rely=0.8, anchor='n')
    def record_data_param_update(self):
        ''' Updates the record data parameters '''
        try:
            self.experiment_dur = int(self.exp_dur_var.get())
        except ValueError:
            print('Invalid Experiment Duration, please input a valid number')
        else:
            self.experiment_name = self.exp_name_var.get()
            # File name for saving the data
            cur_time = rospy.get_time()
            cur_time = datetime.datetime.fromtimestamp(cur_time).strftime('%Y-%m-%d_%H-%M-%S')
            cwd = os.getcwd()
            data_dir = os.path.join(cwd, 'data/pose_data/')
            
            # Check if directory exists, if not, create it
            if not os.path.exists(data_dir):
                os.makedirs(data_dir)
            
            self.file_name = f'data_{self.experiment_name}_{self.experiment_dur}s_{cur_time}.csv'
            self.file_name = os.path.join(data_dir, self.file_name)
    def check_running_cameras(self):
        # check which of the self.cam1_status, self.cam2_status, self.cam3_status are True and return the camera numbers
        cam_nums = []
        if self.cam1_status:
            cam_nums.append(1)
        if self.cam2_status:
            cam_nums.append(2)
        if self.cam3_status:
            cam_nums.append(3)
        return (len(cam_nums), cam_nums)
    
    def record_data(self, cam_num):
        ''' Records the data '''
        print(f'Recording Pose Data for Camera {cam_num}')
        self.cam_first = cam_num
        self.record_data_param_update()
        print(f'File Name: {self.file_name}')
        self.sub1 = rospy.Subscriber(f'/sony_cam{cam_num}/aruco_detect_node/fiducial_transforms', FiducialTransformArray, self.record_single)
        self.is_data_collection_active = True
        rospy.Timer(rospy.Duration(self.experiment_dur), self.stop_data_collection, oneshot=True)
        self.collectected_data = []
    def record_single(self, msg):
        if not self.is_data_collection_active:
            return
        timestamp = rospy.get_time()
        # print the x and y position of the first fiducial from the msg to the console
        print(f'Fiducial ID: {msg.transforms[0].fiducial_id}, Position: ({msg.transforms[0].transform.translation.x}, {msg.transforms[0].transform.translation.y}, {msg.transforms[0].transform.translation.z}), Rotation: ({msg.transforms[0].transform.rotation.x}, {msg.transforms[0].transform.rotation.y}, {msg.transforms[0].transform.rotation.z}, {msg.transforms[0].transform.rotation.w})')
        self.collectected_data.append([timestamp, msg])
    def stop_data_collection(self, event):
        self.is_data_collection_active = False
        self.sub1.unregister()
        self.save_to_csv()
    def save_to_csv(self):
        ''' Saves the data to a CSV file '''
        with open(self.file_name, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time (s)',
                             f'Cam{self.cam_first} Fiducial ID',
                             f'Cam{self.cam_first} Position X',
                             f'Cam{self.cam_first} Position Y',
                             f'Cam{self.cam_first} Position Z',
                             f'Cam{self.cam_first} Rotation X',
                             f'Cam{self.cam_first} Rotation Y',
                              f'Cam{self.cam_first} Rotation Z',
                              f'Cam{self.cam_first} Rotation W'])
            for data in self.collectected_data:
                for msg in data[1].transforms:
                    writer.writerow([data[0], msg.fiducial_id, msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
        print(f'Data saved to {self.file_name}')
    def recall_data(self):
        ''' Recalls the data '''
        print('Recording Pose Data')
        self.record_data_param_update()
        print(f'File Name: {self.file_name}')
        num, cams = self.check_running_cameras()
        print(f'Number of running cameras: {num}, Cameras: {cams}')
        if num == 1:
            first_cam = cams[0]
            self.record_data(first_cam)
        if num == 2:
            first_cam = cams[0]
            second_cam = cams[1]
            self.sub1 = message_filters.Subscriber(f'/sony_cam{first_cam}/aruco_detect_node/fiducial_transforms', FiducialTransformArray)
            self.sub2 = message_filters.Subscriber(f'/sony_cam{second_cam}/aruco_detect_node/fiducial_transforms', FiducialTransformArray)
            self.ats = message_filters.ApproximateTimeSynchronizer([self.sub1, self.sub2], 10, 0.1, allow_headerless=True)
            self.ats.registerCallback(self.record_two_cams)
            self.is_data_collection_active = True
            rospy.Timer(rospy.Duration(self.experiment_dur), self.stop_data_collection2, oneshot=True)
            self.collectected_data = []
        if num == 3:
            print('Recording Pose Data for 3 Cameras')
            self.sub1 = message_filters.Subscriber(f'/sony_cam{cams[0]}/aruco_detect_node/fiducial_transforms', FiducialTransformArray)
            self.sub2 = message_filters.Subscriber(f'/sony_cam{cams[1]}/aruco_detect_node/fiducial_transforms', FiducialTransformArray)
            self.sub3 = message_filters.Subscriber(f'/sony_cam{cams[2]}/aruco_detect_node/fiducial_transforms', FiducialTransformArray)
            self.ats = message_filters.ApproximateTimeSynchronizer([self.sub1, self.sub2, self.sub3], 10, 0.1, allow_headerless=True)
            self.ats.registerCallback(self.record_three_cams)
            self.is_data_collection_active = True
            rospy.Timer(rospy.Duration(self.experiment_dur), self.stop_data_collection3, oneshot=True)
            self.collectected_data = []
    def record_two_cams(self, msg1, msg2):
        if not self.is_data_collection_active:
            return
        timestamp = rospy.get_time()
        # Handle missing messages
        if msg1.transforms and msg2.transforms:
            self.collectected_data.append([timestamp, msg1, msg2])
        elif msg1.transforms:
            self.collectected_data.append([timestamp, msg1, None])
        elif msg2.transforms:
            self.collectected_data.append([timestamp, None, msg2])

    def stop_data_collection2(self, event):
        self.is_data_collection_active = False
        self.sub1.unregister()
        self.sub2.unregister()
        self.save_to_csv2()
    def save_to_csv2(self):
        ''' Save the data from two cameras to a CSV file '''
        with open(self.file_name, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time (s)', 'Cam1 Fiducial ID', 'Cam1 Position X', 'Cam1 Position Y', 'Cam1 Position Z', 'Cam1 Rotation X', 'Cam1 Rotation Y', 'Cam1 Rotation Z', 'Cam1 Rotation W',
                            'Cam2 Fiducial ID', 'Cam2 Position X', 'Cam2 Position Y', 'Cam2 Position Z', 'Cam2 Rotation X', 'Cam2 Rotation Y', 'Cam2 Rotation Z', 'Cam2 Rotation W'])
            for data in self.collectected_data:
                for i in range(len(data[1].transforms)):
                    cam1 = data[1].transforms[i]
                    cam2 = data[2].transforms[i] if i < len(data[2].transforms) else None
                    row = [data[0],
                        cam1.fiducial_id, cam1.transform.translation.x, cam1.transform.translation.y, cam1.transform.translation.z, cam1.transform.rotation.x, cam1.transform.rotation.y, cam1.transform.rotation.z, cam1.transform.rotation.w,
                        cam2.fiducial_id if cam2 else '', cam2.transform.translation.x if cam2 else '', cam2.transform.translation.y if cam2 else '', cam2.transform.translation.z if cam2 else '', cam2.transform.rotation.x if cam2 else '', cam2.transform.rotation.y if cam2 else '', cam2.transform.rotation.z if cam2 else '', cam2.transform.rotation.w if cam2 else '']
                    writer.writerow(row)
        print(f'Data saved to {self.file_name}')
    
    def record_three_cams(self, msg1, msg2, msg3):
        if not self.is_data_collection_active:
            return
        timestamp = rospy.get_time()
        if msg1.transforms and msg2.transforms and msg3.transforms:
            self.collectected_data.append([timestamp, msg1, msg2, msg3])
        elif msg1.transforms and msg2.transforms:
            self.collectected_data.append([timestamp, msg1, msg2, None])
        elif msg1.transforms and msg3.transforms:
            self.collectected_data.append([timestamp, msg1, None, msg3])
        elif msg2.transforms and msg3.transforms:
            self.collectected_data.append([timestamp, None, msg2, msg3])
        elif msg1.transforms:
            self.collectected_data.append([timestamp, msg1, None, None])
        elif msg2.transforms:
            self.collectected_data.append([timestamp, None, msg2, None])
        elif msg3.transforms:
            self.collectected_data.append([timestamp, None, None, msg3])
        else:
            return
        
    def stop_data_collection3(self, event):
        self.is_data_collection_active = False
        self.sub1.unregister()
        self.sub2.unregister()
        self.sub3.unregister()
        self.save_to_csv3()
    def save_to_csv3(self):
        '''Save the data from three cameras to a CSV file'''
        with open(self.file_name, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time (s)', 'Cam1 Fiducial ID', 'Cam1 Position X', 'Cam1 Position Y', 'Cam1 Position Z', 'Cam1 Rotation X', 'Cam1 Rotation Y', 'Cam1 Rotation Z', 'Cam1 Rotation W',
                            'Cam2 Fiducial ID', 'Cam2 Position X', 'Cam2 Position Y', 'Cam2 Position Z', 'Cam2 Rotation X', 'Cam2 Rotation Y', 'Cam2 Rotation Z', 'Cam2 Rotation W',
                            'Cam3 Fiducial ID', 'Cam3 Position X', 'Cam3 Position Y', 'Cam3 Position Z', 'Cam3 Rotation X', 'Cam3 Rotation Y', 'Cam3 Rotation Z', 'Cam3 Rotation W'])
            for data in self.collectected_data:
                for i in range(len(data[1].transforms)):
                    cam1 = data[1].transforms[i]
                    cam2 = data[2].transforms[i] if i < len(data[2].transforms) else None
                    cam3 = data[3].transforms[i] if i < len(data[3].transforms) else None
                    row = [data[0],
                        cam1.fiducial_id, cam1.transform.translation.x, cam1.transform.translation.y, cam1.transform.translation.z, cam1.transform.rotation.x, cam1.transform.rotation.y, cam1.transform.rotation.z, cam1.transform.rotation.w,
                        cam2.fiducial_id if cam2 else '', cam2.transform.translation.x if cam2 else '', cam2.transform.translation.y if cam2 else '', cam2.transform.translation.z if cam2 else '', cam2.transform.rotation.x if cam2 else '', cam2.transform.rotation.y if cam2 else '', cam2.transform.rotation.z if cam2 else '', cam2.transform.rotation.w if cam2 else '',
                        cam3.fiducial_id if cam3 else '', cam3.transform.translation.x if cam3 else '', cam3.transform.translation.y if cam3 else '', cam3.transform.translation.z if cam3 else '', cam3.transform.rotation.x if cam3 else '', cam3.transform.rotation.y if cam3 else '', cam3.transform.rotation.z if cam3 else '', cam3.transform.rotation.w if cam3 else '']
                    writer.writerow(row)
        print(f'Data saved to {self.file_name}')
    def create_middle_second_bottom_frame(self)-> None:
        ''' Creates the bottom frame in the middle second frame '''
        self.middle_second_bottom_frame = ctk.CTkFrame(self.middle_second_frame)
        self.middle_second_bottom_frame.place(relx=0.5, rely=(self.ver_space*3)+self.label_height+self.frame_height+0.1, relwidth=self.frame_width, relheight=self.frame_height, anchor='n')
        self.create_middle_second_bottom_frame_widgets()
    def create_middle_second_bottom_frame_widgets(self)-> None:
        ''' Creates the widgets in the bottom frame in the middle second frame '''
        self.middle_second_bottom_frame_label = ctk.CTkLabel(self.middle_second_bottom_frame, text='PLOT DATA')
        self.middle_second_bottom_frame_label.place(relx=0.5, rely=0.1, anchor='n')
        self.middle_second_bottom_frame_button = ctk.CTkButton(self.middle_second_bottom_frame, text='PLOT - OVERLAP', command=lambda:self.plot_data(True))
        self.middle_second_bottom_frame_button.place(relx=0.5, rely=0.5, anchor='n')
        self.middle_second_bottom_frame_button = ctk.CTkButton(self.middle_second_bottom_frame, text='PLOT - SEPARATE', command=lambda:self.plot_data(False))
        self.middle_second_bottom_frame_button.place(relx=0.5, rely=0.8, anchor='n')

    def plot_data(self, overlap: bool):
        ''' Plots the data '''
        print('Experiment name:', self.experiment_name)
        print('File name:', self.file_name)
        print('Experiment duration:', self.experiment_dur)
        print('Plotting Data')

        data = pd.read_csv(self.file_name)

        def adjust_positions(data, cam_prefix):
            if f'{cam_prefix} Position X' not in data.columns or data[f'{cam_prefix} Position X'].empty:
                print(f"No data available for {cam_prefix}.")
                return
            for axis in ['X', 'Y', 'Z']:
                col_name = f'{cam_prefix} Position {axis}'
                data[col_name] -= data[col_name].iloc[0]
            for axis in ['X', 'Y', 'Z', 'W']:
                col_name = f'{cam_prefix} Rotation {axis}'
                data[col_name] -= data[col_name].iloc[0]


        # Check for available cameras
        available_cameras = []
        for cam_num in range(1, 4):
            col_name = f'Cam{cam_num} Position X'
            if col_name in data.columns and not data[col_name].empty:
                available_cameras.append(cam_num)
                adjust_positions(data, f'Cam{cam_num}')

        if not available_cameras:
            print("No camera data available.")
            return

        # Convert Unix timestamp to seconds relative to the start of the experiment
        start_time = data['Time (s)'].iloc[0]
        data['Time (s)'] = (data['Time (s)'] - start_time)

        def plot_axis(ax, x_data, y_data, label, color, linestyle='-'):
            ax.plot(x_data, y_data, label=label, color=color, linestyle=linestyle)

        colors = ['red', 'green', 'blue']
        linestyles = ['-', '-', '-']
        axis_labels = ['X', 'Y', 'Z']

        fig, axs = plt.subplots(3, 1, figsize=(15, 15))

        for i, axis in enumerate(axis_labels):
            for cam_num in available_cameras:
                col_name = f'Cam{cam_num} Position {axis}'
                plot_axis(axs[i], data['Time (s)'].values, data[col_name].values, f'Cam{cam_num} Position {axis}', colors[cam_num-1], linestyles[cam_num-1])
            axs[i].set_title(f'{axis} Axis Displacement - {self.experiment_name}')
            axs[i].set_xlabel('Time (s)')
            axs[i].set_ylabel('Displacement (mm)')
            axs[i].legend()
            axs[i].grid(True, which='both')
            axs[i].minorticks_on()
            axs[i].xaxis.set_major_locator(MaxNLocator(integer=True))
            axs[i].yaxis.set_major_locator(MaxNLocator(integer=True))
            axs[i].set_yticklabels([f'{x * 1000:.2f}' for x in axs[i].get_yticks()])
            axs[i].set_xticklabels([f'{x:.0f}' for x in axs[i].get_xticks()])

        file_name = self.file_name.replace('.csv', '.png')
        plt.savefig(file_name)
        print(f'Plot saved to {file_name}')
        plt.show()

    def create_middle_first_frame(self)-> None:
        ''' Creates the middle frame for setting system parameters '''
        self.middle_first_frame = tk.Frame(self, bg=themes['blue'][2])
        self.middle_first_frame.place(relx=0.25, rely=0, relwidth=0.25, relheight=1)
        self.create_middle_first_top_frame()
        self.create_middle_first_center_frame()
        self.create_middle_first_bottom_frame()
        self.exit_button_frame()
    def create_middle_first_top_frame(self)-> None:
        ''' Creates the top frame in the middle first frame '''
        self.middle_first_top_frame = ctk.CTkFrame(self.middle_first_frame)
        self.middle_first_top_frame.place(relx=0.5, rely=self.ver_space, relwidth=self.frame_width, relheight=self.label_height, anchor='n')
        self.create_middle_first_top_frame_widgets()
    def create_middle_first_top_frame_widgets(self)-> None:
        ''' Creates the widgets in the top frame in the middle first frame '''
        self.middle_first_top_frame_label = ctk.CTkLabel(self.middle_first_top_frame, text='SYSTEM PARAMETERS')
        self.middle_first_top_frame_label.place(relx=0.5, rely=0.5, anchor='center')
    def create_middle_first_center_frame(self)-> None:
        ''' Creates the center frame in the middle first frame '''
        self.middle_first_center_frame = ctk.CTkFrame(self.middle_first_frame)
        self.middle_first_center_frame.place(relx=0.5, rely=(self.ver_space*2)+self.label_height, relwidth=self.frame_width, relheight=self.frame_height, anchor='n')
        self.create_middle_first_center_frame_widgets()
    def create_middle_first_bottom_frame(self)-> None:
        ''' Creates the bottom frame in the middle first frame '''
        self.middle_first_bottom_frame = ctk.CTkFrame(self.middle_first_frame)
        self.middle_first_bottom_frame.place(relx=0.5, rely=(self.ver_space*3)+self.label_height+self.frame_height, relwidth=self.frame_width, relheight=self.frame_height, anchor='n')
        self.create_middle_first_bottom_frame_widgets()
    def exit_button_frame(self)-> None:
        ''' Creates the exit button frame '''
        self.exit_button = ctk.CTkButton(self.middle_first_frame, text='EXIT', command=self.on_closing, fg_color=themes["red"])
        self.exit_button.place(relx=0.5, rely=(self.ver_space*4)+self.label_height+(self.frame_height*2), relwidth=self.frame_width, relheight=self.label_height, anchor='n')
    def create_middle_first_center_frame_widgets(self)-> None:
        '''Frame for setting Calibration parameters'''
        self.calib_label = ctk.CTkLabel(self.middle_first_center_frame, text='CALIBRATION PARAMETERS')
        self.calib_square_size_label = ctk.CTkLabel(self.middle_first_center_frame, text='Square Size (m):')
        self.calib_square_size_entry = ctk.CTkEntry(self.middle_first_center_frame, textvariable=self.sq_size_var)
        self.calib_board_size_label = ctk.CTkLabel(self.middle_first_center_frame, text='Board Size:')
        self.calib_board_size_entry = ctk.CTkEntry(self.middle_first_center_frame, textvariable=self.board_size_var)
        self.calib_update_btn = ctk.CTkButton(self.middle_first_center_frame, text='UPDATE', command=self.update_calib_params)
        self.calib_label.place(relx=0.5, rely=0.1, anchor='n')
        self.calib_square_size_label.place(relx=0.1, rely=0.3)
        self.calib_square_size_entry.place(relx=0.8, rely=0.3, anchor='n', relwidth=0.3)
        self.calib_board_size_label.place(relx=0.1, rely=0.5)
        self.calib_board_size_entry.place(relx=0.8, rely=0.5, anchor='n', relwidth=0.3)
        self.calib_update_btn.place(relx=0.5, rely=0.7, anchor='n')
    def update_calib_params(self):
        ''' Updates the calibration parameters '''
        self.square_size = self.calib_square_size_entry.get()
        self.board_size = self.calib_board_size_entry.get()
        print(f'Square Size: {self.square_size}, Board Size: {self.board_size}')
        self.calib_update_btn.configure(text='UPDATED', fg_color='green')
    def create_middle_first_bottom_frame_widgets(self)-> None:
        '''Update the ARUCO parameters'''
        self.aruco_label = ctk.CTkLabel(self.middle_first_bottom_frame, text='DETECTION PARAMETERS')
        self.aruco_marker_dim_label = ctk.CTkLabel(self.middle_first_bottom_frame, text='Marker Size (m):')
        self.aruco_marker_dim_entry = ctk.CTkEntry(self.middle_first_bottom_frame, textvariable=self.marker_dim_var)
        self.aruco_marker_dict_label = ctk.CTkLabel(self.middle_first_bottom_frame, text='Marker Dictionary:')
        self.aruco_marker_dict_entry = ctk.CTkEntry(self.middle_first_bottom_frame, textvariable=self.marker_dict_var)
        self.aruco_update_btn = ctk.CTkButton(self.middle_first_bottom_frame, text='UPDATE', command=self.update_aruco_params)
        self.aruco_label.place(relx=0.5, rely=0.1, anchor='n')
        self.aruco_marker_dim_label.place(relx=0.1, rely=0.3)
        self.aruco_marker_dim_entry.place(relx=0.8, rely=0.3, anchor='n', relwidth=0.3)
        self.aruco_marker_dict_label.place(relx=0.1, rely=0.5)
        self.aruco_marker_dict_entry.place(relx=0.8, rely=0.5, anchor='n', relwidth=0.3)
        self.aruco_update_btn.place(relx=0.5, rely=0.7, anchor='n')
    def update_aruco_params(self):
        ''' Updates the ARUCO parameters '''
        self.marker_dim = self.aruco_marker_dim_entry.get()
        self.marker_dict = self.aruco_marker_dict_entry.get()
        print(f'Marker Dimension: {self.marker_dim}, Marker Dictionary: {self.marker_dict}')
        self.aruco_update_btn.configure(text='UPDATED', fg_color='green')
    def on_closing(self):
        ''' Stops all the processes and closes the GUI '''
        self.is_detection_active = False
        self.is_data_collection_active = False
        print('Stopping all processes')
        
        # Make a copy of the keys to avoid modifying the dictionary while iterating
        processes = list(self.running_processes.keys())
        for process in processes:
            self.cleanup_process(process)
        
        self.destroy()

    def cleanup_process(self, process_name):
        ''' Stops the process '''
        try:
            self.running_processes[process_name].shutdown()
            del self.running_processes[process_name]
        except KeyError:
            print(f'{process_name} not running')



    def create_left_frame(self)-> None:
        ''' Creates the left frame to start Camera and Detection '''
        self.left_frame = tk.Frame(self, bg=themes[COLOR_SELECT][0])
        self.left_frame.place(relx=0, rely=0, relwidth=0.25, relheight=1)
        self.create_left_top_frame()
        self.create_left_center_first_frame()
        self.create_left_center_second_frame()
        self.create_left_bottom_frame()
    def create_left_top_frame(self)-> None:
        ''' Creates the top frame in the left frame '''
        self.left_top_frame = ctk.CTkFrame(self.left_frame)
        self.left_top_frame.place(relx=0.5, rely=self.ver_space, relwidth=self.frame_width, relheight=self.label_height, anchor='n')
        self.create_left_top_frame_widgets()
    def create_left_center_first_frame(self)-> None:
        ''' Starting Cameras '''
        self.left_center_first_frame = ctk.CTkFrame(self.left_frame)
        self.left_center_first_frame.place(relx=0.5, rely=(self.ver_space*2)+self.label_height, relwidth=self.frame_width, relheight=self.frame_height, anchor='n')
        self.create_left_center_first_frame_widgets()
    def create_left_center_second_frame(self)-> None:
        ''' Starting Detection '''
        self.left_center_second_frame = ctk.CTkFrame(self.left_frame)
        self.left_center_second_frame.place(relx=0.5, rely=(self.ver_space*3)+self.label_height+self.frame_height, relwidth=self.frame_width, relheight=0.15, anchor='n')
        self.create_left_center_second_frame_widgets()
    def create_left_bottom_frame(self)-> None:
        ''' Starting Data Collection '''
        self.left_bottom_frame = ctk.CTkFrame(self.left_frame)
        self.left_bottom_frame.place(relx=0.5, rely=(self.ver_space*4)+self.label_height+self.frame_height+0.15, relwidth=self.frame_width, relheight=0.20, anchor='n')
        self.create_left_bottom_frame_widgets()
    def create_left_top_frame_widgets(self)-> None:
        ''' Creates the widgets in the top frame in the left frame '''
        self.left_top_frame_label = ctk.CTkLabel(self.left_top_frame, text='START CAMERA & DETECTION')
        self.left_top_frame_label.place(relx=0.5, rely=0.5, anchor='center')
    def create_left_center_first_frame_widgets(self)-> None:
        ''' Starts individual cameras'''
        self.left_start_cam_label = ctk.CTkLabel(self.left_center_first_frame, text='Start Camera')
        self.left_start_cam1_button = ctk.CTkButton(self.left_center_first_frame, text='  1  ', command=lambda:self.cam_btn_event(1), width=2)
        self.left_start_cam2_button = ctk.CTkButton(self.left_center_first_frame, text='  2  ', command=lambda:self.cam_btn_event(2), width=2)
        self.left_start_cam3_button = ctk.CTkButton(self.left_center_first_frame, text='  3  ', command=lambda:self.cam_btn_event(3), width=2)

        self.left_start_view_label = ctk.CTkLabel(self.left_center_first_frame, text='View Camera')
        self.left_view1_button = ctk.CTkButton(self.left_center_first_frame, text='  1  ', command=lambda:self.view_btn_event(1), width=2, fg_color='gray')
        self.left_view2_button = ctk.CTkButton(self.left_center_first_frame, text='  2  ', command=lambda:self.view_btn_event(2), width=2, fg_color='gray')
        self.left_view3_button = ctk.CTkButton(self.left_center_first_frame, text='  3  ', command=lambda:self.view_btn_event(3), width=2, fg_color='gray')

        self.left_startnview_label = ctk.CTkLabel(self.left_center_first_frame, text='Start & View')
        self.left_startnview1_button = ctk.CTkButton(self.left_center_first_frame, text='  1  ', command=lambda:self.cam_view_btn_event(1), width=2)
        self.left_startnview2_button = ctk.CTkButton(self.left_center_first_frame, text='  2  ' , command=lambda:self.cam_view_btn_event(2), width=2)
        self.left_startnview3_button = ctk.CTkButton(self.left_center_first_frame, text='  3  ', command=lambda:self.cam_view_btn_event(3), width=2)

        v_space1 = 0.15
        v_space2 = 0.42
        v_space3 = 0.69
        h_space = 0.25

        start1_pos = 0.52
        start2_pos = 0.69
        start3_pos = 0.86
        self.left_start_cam_label.place(relx=h_space, rely=v_space1, anchor='n')
        self.left_start_cam1_button.place(relx=start1_pos, rely=v_space1, anchor='n')
        self.left_start_cam2_button.place(relx=start2_pos, rely=v_space1, anchor='n')
        self.left_start_cam3_button.place(relx=start3_pos, rely=v_space1, anchor='n')
        
        self.left_start_view_label.place(relx=h_space, rely=v_space2, anchor='n')
        self.left_view1_button.place(relx=start1_pos, rely=v_space2, anchor='n')
        self.left_view2_button.place(relx=start2_pos, rely=v_space2, anchor='n')
        self.left_view3_button.place(relx=start3_pos, rely=v_space2, anchor='n')
        
        self.left_startnview_label.place(relx=h_space, rely=v_space3, anchor='n')
        self.left_startnview1_button.place(relx=start1_pos, rely=v_space3, anchor='n')
        self.left_startnview2_button.place(relx=start2_pos, rely=v_space3, anchor='n')
        self.left_startnview3_button.place(relx=start3_pos, rely=v_space3, anchor='n')
    def create_left_center_second_frame_widgets(self)-> None:
        ''' Calibrate Cameras '''
        self.left_calib_cam_label = ctk.CTkLabel(self.left_center_second_frame, text='CALIBRATE CAMERA')
        self.left_calib_cam1_button = ctk.CTkButton(self.left_center_second_frame, text='1', command=lambda:self.calibrate_cam_btn_event(1))
        self.left_calib_cam2_button = ctk.CTkButton(self.left_center_second_frame, text='2', command=lambda:self.calibrate_cam_btn_event(2))
        self.left_calib_cam3_button = ctk.CTkButton(self.left_center_second_frame, text='3', command=lambda:self.calibrate_cam_btn_event(3))
        self.left_calib_cam_label.place(relx=0.5, rely=0.1, anchor='n')
        self.left_calib_cam1_button.place(relx=0.2, rely=0.5, anchor='n', relwidth=0.2)
        self.left_calib_cam2_button.place(relx=0.5, rely=0.5, anchor='n', relwidth=0.2)
        self.left_calib_cam3_button.place(relx=0.8, rely=0.5, anchor='n', relwidth=0.2)
        
    def create_left_bottom_frame_widgets(self)-> None:
        self.left_start_detect_label = ctk.CTkLabel(self.left_bottom_frame, text='START DETECTION')
        self.left_detect1_button = ctk.CTkButton(self.left_bottom_frame, text='1', command=lambda:self.start_detect_btn_event(1))
        self.left_detect2_button = ctk.CTkButton(self.left_bottom_frame, text='2', command=lambda:self.start_detect_btn_event(2))
        self.left_detect3_button = ctk.CTkButton(self.left_bottom_frame, text='3', command=lambda:self.start_detect_btn_event(3))
        self.left_detectall_button = ctk.CTkButton(self.left_bottom_frame, text='ALL CAMERAS', command=self.start_detection_all, fg_color='gray')
        
        self.left_start_detect_label.place(relx=0.5, rely=0.08, anchor='n')
        self.left_detect1_button.place(relx=0.2, rely=0.34, anchor='n', relwidth=0.2)
        self.left_detect2_button.place(relx=0.5, rely=0.34, anchor='n', relwidth=0.2)
        self.left_detect3_button.place(relx=0.8, rely=0.34, anchor='n', relwidth=0.2)
        self.left_detectall_button.place(relx=0.5, rely=0.66, anchor='n')
    def start_detect_btn_event(self, cam_num):
        ''' Starts detection for the selected camera '''
        rospy.loginfo(f'Starting detection for camera {cam_num}')
        if self.running_processes.get(f'sony_cam{cam_num}_detect_driver') is not None:
            # i.e., the detection process is running, stop the detection process
            try:
                self.cleanup_process(f'sony_cam{cam_num}_detect_driver')
            except KeyError:
                rospy.logerr(f'Detection process for camera {cam_num} not found')
            else:
                print(f'Detection for camera {cam_num} stopped')
                self._ui_detect_cam_btn(cam_num, "IDLE")
                try:
                    self.cleanup_process(f'sony_cam{cam_num}_cam_driver')
                except KeyError:
                    rospy.logerr(f'Camera {cam_num} not found')
                else:
                    print(f'Camera {cam_num} stopped')
                    self._ui_start_cam_btn(cam_num, "IDLE")
        else:
            # i.e., the detection process is not running, start the detection process
            # check if the camera is running or not
            if self.running_processes.get(f'sony_cam{cam_num}_cam_driver') is not None:
                # i.e., the camera is running, start the detection process
                try:
                    self.start_detect_process(cam_num)
                except roslaunch.RLException as e:
                    rospy.logerr(f'Error starting detection for camera {cam_num}: {e}')
                else:
                    print(f'Detection for camera {cam_num} started')
                    self._ui_detect_cam_btn(cam_num, "RUNNING")
            else:
                # i.e., the camera is not running, start the camera and then the detection process
                try:
                    self.start_camera_process(cam_num)
                    self.start_detect_process(cam_num)
                except roslaunch.RLException as e:
                    rospy.logerr(f'Error starting detection for camera {cam_num}: {e}')
                else:
                    print(f'Detection for camera {cam_num} started')
                    self._ui_detect_cam_btn(cam_num, "RUNNING")
                    self._ui_start_cam_btn(cam_num, "RUNNING")
    def start_detect_process(self, cam_num):
        ''' Starts the detection process '''
        self.marker_dict = self.marker_dict_var.get()
        self.marker_dim = self.marker_dim_var.get()
        detect_launch_args = [
            f'{self.detect_launch_file}',
            f'launch_nuc:=sony_cam{cam_num}',
            f'dictionary:={self.marker_dict}',
            f'fiducial_len:={self.marker_dim}']
        detect_roslaunch_file = [(
            roslaunch.rlutil.resolve_launch_arguments(detect_launch_args)[0],
            detect_launch_args[1:])]
        detect_driver = roslaunch.parent.ROSLaunchParent(self.uuid, detect_roslaunch_file)
        detect_driver.start()
        self.running_processes[f'sony_cam{cam_num}_detect_driver'] = detect_driver
        rospy.loginfo(f'Detection for camera {cam_num} started')
        rospy.sleep(0.5)
    def start_detection_all(self):
        ''' Starts detection for all the cameras '''
        for cam_num in range(1, 4):
            self.start_detect_btn_event(cam_num)
            



        
    def cam_btn_event(self, cam_num):
        ''' Starts the camera '''
        try:
            if self.running_processes.get(f'sony_cam{cam_num}_cam_driver') is not None:
                # i.e., the camera is running, stop the camera
                rospy.loginfo(f'Stopping camera {cam_num}')
                # check and stop the detection process if running
                if self.running_processes.get(f'sony_cam{cam_num}_detect_driver') is not None:
                    # i.e., the detection process is running, stop the detection process
                    rospy.loginfo(f'Stopping detection for camera {cam_num}')
                    try:
                        self.cleanup_process(f'sony_cam{cam_num}_detect_driver')
                    except KeyError:
                        rospy.logerr(f'Detection process for camera {cam_num} not found')
                    else:
                        print(f'Camera {cam_num} detection stopped')
                        # update the detection status in the right pane
                        self._ui_detect_cam_btn(cam_num, "IDLE")
                if self.running_processes.get(f'sony_cam{cam_num}_view_driver') is not None:
                    # i.e., the view process is running, stop the view process
                    rospy.loginfo(f'Stopping view for camera {cam_num}')
                    try:
                        self.cleanup_process(f'sony_cam{cam_num}_view_driver')
                    except KeyError:
                        rospy.logerr(f'View process for camera {cam_num} not found')
                    else:
                        # update the view status in the right pane
                        print(f'Camera {cam_num} view stopped')
                        self._ui_start_cam_btn(cam_num, "IDLE")
                        self._ui_view_cam_btn(cam_num, "IDLE")
                        self._ui_start_view_cam_btn(cam_num, "IDLE")
                else:
                    # i.e., view and detect processes are not running, but camera is running so stop the camera
                    try:
                        self.cleanup_process(f'sony_cam{cam_num}_cam_driver')
                    except KeyError:
                        rospy.logerr(f'Camera {cam_num} not found')
                    else:
                        print(f'Camera {cam_num} stopped')
                        self._ui_start_cam_btn(cam_num, "IDLE")
            else:
                # i.e., the camera is not running, start the camera
                rospy.loginfo(f'Starting camera {cam_num}')
                try:
                    self.start_camera_process(cam_num)
                except roslaunch.RLException as e:
                    rospy.logerr(f'Error starting camera {cam_num}: {e}')
                else:
                    rospy.loginfo(f'Camera {cam_num} started')
                    self._ui_start_cam_btn(cam_num, "RUNNING")
        except rospy.ROSInterruptException:
            rospy.logerr('ROS Interrupted')
    def start_camera_process(self, cam_num):
        ''' Starts the camera process '''
        cam_launch_args = [
            f'{self.cam_launch_file}',
            f'launch_nuc:=sony_cam{cam_num}',
            f'image_width:={self.image_width}',
            f'image_height:={self.image_height}']
        cam_roslaunch_file = [(
            roslaunch.rlutil.resolve_launch_arguments(cam_launch_args)[0],
            cam_launch_args[1:])]
        cam_driver = roslaunch.parent.ROSLaunchParent(self.uuid, cam_roslaunch_file)
        cam_driver.start()
        self.running_processes[f'sony_cam{cam_num}_cam_driver'] = cam_driver
        rospy.loginfo(f'Camera {cam_num} started')
        rospy.sleep(0.5)
    def cam_view_btn_event(self, cam_num):
        rospy.loginfo(f'Starting view for camera {cam_num}')
        # checking if the camera is running
        if self.running_processes.get(f'sony_cam{cam_num}_cam_driver') is not None:
            # i.e., the camera is running
            if self.running_processes.get(f'sony_cam{cam_num}_view_driver') is not None:
                # i.e., the view process is running, stop the camera and view processes 
                # and update the statuses in the right pane
                try:
                    self.cleanup_process(f'sony_cam{cam_num}_view_driver')
                    self.cleanup_process(f'sony_cam{cam_num}_cam_driver')
                except KeyError:
                    rospy.logerr(f'View process for camera {cam_num} not found')
                else:
                    print(f'Camera {cam_num} view stopped')
                    # stop the camera driver as well
                    self._ui_start_cam_btn(cam_num, "IDLE")
                    self._ui_view_cam_btn(cam_num, "IDLE")
                    self._ui_start_view_cam_btn(cam_num, "IDLE")
            else:
                # i.e., the view process is not running, start the view process
                try:
                    self.start_view_process(cam_num)
                except roslaunch.RLException as e:
                    rospy.logerr(f'Error starting view for camera {cam_num}: {e}')
                else:
                    print(f'Camera {cam_num} view started')
                    self._ui_view_cam_btn(cam_num, "RUNNING")
                    self._ui_start_view_cam_btn(cam_num, "RUNNING")
        else:
            # i.e., the camera is not running
            # first start the camera and then the view driver
            try:
                self.start_camera_process(cam_num)
                self.start_view_process(cam_num)
            except roslaunch.RLException as e:
                rospy.logerr(f'Error starting view for camera {cam_num}: {e}')
            else:
                print(f'Camera {cam_num} view started')
                self._ui_start_cam_btn(cam_num, "RUNNING")
                self._ui_view_cam_btn(cam_num, "RUNNING")
                self._ui_start_view_cam_btn(cam_num, "RUNNING")
    def view_btn_event(self, cam_num):
        ''' Starts the view camera process '''
        # check if the camera is running
        if self.running_processes.get(f'sony_cam{cam_num}_cam_driver') is not None:
            # i.e., the camera is running
            if self.running_processes.get(f'sony_cam{cam_num}_view_driver') is not None:
                # i.e., the view process is running, stop the view process
                try:
                    self.cleanup_process(f'sony_cam{cam_num}_view_driver')
                except KeyError:
                    rospy.logerr(f'View process for camera {cam_num} not found')
                else:
                    print(f'Camera {cam_num} view stopped')
                    self._ui_view_cam_btn(cam_num, "IDLE")
                    self._ui_start_view_cam_btn(cam_num, "IDLE")
            else:
                # i.e., the view process is not running, start the view process
                try:
                    self.start_view_process(cam_num)
                except roslaunch.RLException as e:
                    rospy.logerr(f'Error starting view for camera {cam_num}: {e}')
                else:
                    print(f'Camera {cam_num} view started')
                    self._ui_view_cam_btn(cam_num, "RUNNING")
                    self._ui_start_view_cam_btn(cam_num, "RUNNING")
        else:
            # i.e., the camera is not running
            rospy.logerr(f'Camera {cam_num} not running')
    def calibrate_cam_btn_event(self, cam_num):
        ''' Calibrates the camera '''
        if self.running_processes.get(f'sony_cam{cam_num}_calib_driver') is not None:
            # i.e., the calibration process is running, stop the calibration process
            try:
                self.cleanup_process(f'sony_cam{cam_num}_calib_driver')
            except KeyError:
                rospy.logerr(f'Calibration process for camera {cam_num} not found')
            else:
                print(f'Camera {cam_num} calibration stopped')
                self._ui_calib_cam_btn(cam_num, "IDLE")
                try:
                    self.cleanup_process(f'sony_cam{cam_num}_cam_driver')
                    self.cleanup_process(f'sony_cam{cam_num}_view_driver')
                    
                except KeyError:
                    rospy.logerr(f'Camera {cam_num} not found')
                else:
                    print(f'Camera {cam_num} stopped')
                    self._ui_start_cam_btn(cam_num, "IDLE")
                    self._ui_view_cam_btn(cam_num, "IDLE")
                    self._ui_start_view_cam_btn(cam_num, "IDLE")
        else:
            # i.e., the calibration process is not running, start the calibration process
            # but first start the camera node first and put it into the processes dictionary
            try:
                self.start_calibration_process(cam_num)
            except roslaunch.RLException as e:
                rospy.logerr(f'Error starting calibration for camera {cam_num}: {e}')
            else:
                print(f'Camera {cam_num} calibration started')
                self._ui_calib_cam_btn(cam_num, "RUNNING")
    def start_calibration_process(self, cam_num):
        ''' Starts the calibration process '''
        self.board_size = self.board_size_var.get()
        self.square_size = self.sq_size_var.get()
        if self.running_processes.get(f'sony_cam{cam_num}_cam_driver') is None:
            # i.e., the camera is not running, start the camera
            try:
                self.start_camera_process(cam_num)
            except roslaunch.RLException as e:
                rospy.logerr(f'Error starting camera {cam_num}: {e}')
            else:
                print(f'Camera {cam_num} started')
                self._ui_start_cam_btn(cam_num, "RUNNING")
        calib_launch_args = [
            f'{self.cam_calib_launch_file}',
            f'camera_name:=sony_cam{cam_num}',
            f'cb_size:={self.board_size}',
            f'cb_square:={self.square_size}']
        calib_roslaunch_file = [(
            roslaunch.rlutil.resolve_launch_arguments(calib_launch_args)[0],
            calib_launch_args[1:])]
        calib_driver = roslaunch.parent.ROSLaunchParent(self.uuid, calib_roslaunch_file)
        calib_driver.start()
        self.running_processes[f'sony_cam{cam_num}_calib_driver'] = calib_driver
        rospy.loginfo(f'Calibration for camera {cam_num} started')
        rospy.sleep(0.5)



    def start_view_process(self, cam_num):
        ''' Starts the view process '''
        view_launch_args = [
            f'{self.cam_view_launch_file}',
            f'launch_nuc:=sony_cam{cam_num}']
        view_roslaunch_file = [(
            roslaunch.rlutil.resolve_launch_arguments(view_launch_args)[0],
            view_launch_args[1:])]
        view_driver = roslaunch.parent.ROSLaunchParent(self.uuid, view_roslaunch_file)
        view_driver.start()
        self.running_processes[f'sony_cam{cam_num}_view_driver'] = view_driver
        rospy.loginfo(f'View for camera {cam_num} started')
        rospy.sleep(0.5)
    def _ui_start_cam_btn(self, cam_num, status):
        ''' Updates the camera status in the UI '''
        if cam_num == 1:
            if status == "RUNNING":
                self.left_start_cam1_button.configure(fg_color='green')
                self.cam1_status = True
            else:
                self.left_start_cam1_button.configure(fg_color=themes['blue'][0])
                self.cam1_status = False
        elif cam_num == 2:
            if status == "RUNNING":
                self.left_start_cam2_button.configure(fg_color='green')
                self.cam2_status = True
            else:
                self.left_start_cam2_button.configure(fg_color=themes['blue'][0])
                self.cam2_status = False
        elif cam_num == 3:
            if status == "RUNNING":
                self.left_start_cam3_button.configure(fg_color='green')
                self.cam3_status = True
            else:
                self.left_start_cam3_button.configure(fg_color=themes['blue'][0])
                self.cam3_status = False
    def _ui_view_cam_btn(self, cam_num, status):
        ''' Updates the view status in the UI '''
        if cam_num == 1:
            if status == "RUNNING":
                self.left_view1_button.configure(fg_color='green')
            else:
                self.left_view1_button.configure(fg_color='gray')
        elif cam_num == 2:
            if status == "RUNNING":
                self.left_view2_button.configure(fg_color='green')
            else:
                self.left_view2_button.configure(fg_color='gray')
        elif cam_num == 3:
            if status == "RUNNING":
                self.left_view3_button.configure(fg_color='green')
            else:
                self.left_view3_button.configure(fg_color='gray')
    def _ui_start_view_cam_btn(self, cam_num, status):
        ''' Updates the start view status in the UI '''
        if cam_num == 1:
            if status == "RUNNING":
                self.left_startnview1_button.configure(fg_color='green')
            else:
                self.left_startnview1_button.configure(fg_color=themes['blue'][0])
        elif cam_num == 2:
            if status == "RUNNING":
                self.left_startnview2_button.configure(fg_color='green')
            else:
                self.left_startnview2_button.configure(fg_color=themes['blue'][0])
        elif cam_num == 3:
            if status == "RUNNING":
                self.left_startnview3_button.configure(fg_color='green')
            else:
                self.left_startnview3_button.configure(fg_color=themes['blue'][0])
    def _ui_calib_cam_btn(self, cam_num, status):
        ''' Updates the calibration status in the UI '''
        if cam_num == 1:
            if status == "RUNNING":
                self.left_calib_cam1_button.configure(fg_color='green')
            else:
                self.left_calib_cam1_button.configure(fg_color=themes['blue'][0])
        elif cam_num == 2:
            if status == "RUNNING":
                self.left_calib_cam2_button.configure(fg_color='green')
            else:
                self.left_calib_cam2_button.configure(fg_color=themes['blue'][0])
        elif cam_num == 3:
            if status == "RUNNING":
                self.left_calib_cam3_button.configure(fg_color='green')
            else:
                self.left_calib_cam3_button.configure(fg_color=themes['blue'][0])
    def _ui_detect_cam_btn(self, cam_num, status):
        ''' Updates the detection status in the UI '''
        if cam_num == 1:
            if status == "RUNNING":
                self.left_detect1_button.configure(fg_color='green')
                self.detect1_status = True
            else:
                self.left_detect1_button.configure(fg_color=themes['blue'][0])
                self.detect1_status = False
        elif cam_num == 2:
            if status == "RUNNING":
                self.left_detect2_button.configure(fg_color='green')
                self.detect2_status = True
            else:
                self.left_detect2_button.configure(fg_color=themes['blue'][0])
                self.detect2_status = False
        elif cam_num == 3:
            if status == "RUNNING":
                self.left_detect3_button.configure(fg_color='green')
                self.detect3_status = True
            else:
                self.left_detect3_button.configure(fg_color=themes['blue'][0])
                self.detect3_status = False
if __name__ == "__main__":
    rospy.init_node('fin_gui', anonymous=False)
    app = NodeGUI()
    app.mainloop()