#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import datetime
import tkinter as tk
import customtkinter as ctk
import pandas as pd
import numpy as np
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
    
        self.image_width = '640'
        self.image_height = '480'
        # self.image_height = '1080'
        # self.image_width = '1920'
        
        # self.image_width = '1280'
        # self.image_height = '720'
        
        # self.image_width = '854'
        # self.image_height = '480'
        
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

        self.tag_size = '0.03' # Dimension in meters for AprilTag
        self.tag_family = "tag36h11" #  AprilTag family
        self.tag_size_var = tk.StringVar(self, self.tag_size)
        self.tag_family_var = tk.StringVar(self, self.tag_family)

        self.cam_pkg = 'sony_cam'
        self.detect_pkg = 'tesol_detect'
        try:
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            self.cam_launch_path = rospkg.RosPack().get_path(self.cam_pkg) + '/launch/'
            self.detect_launch_path = rospkg.RosPack().get_path(self.detect_pkg) + '/launch/'
            self.cam_launch_file = f'{self.cam_launch_path}use_cam.launch'
            self.cam_view_launch_file = f'{self.cam_launch_path}use_viewcam.launch'
            self.cam_calib_launch_file = f'{self.cam_launch_path}calib.launch'
            self.detect_launch_file = f'{self.detect_launch_path}use_tesol_april.launch'
        except rospkg.common.ResourceNotFound:
            print('ROS packages not found')
            self.cam_launch_file = None
            self.cam_view_launch_file = None
            self.cam_calib_launch_file = None
            self.detect_launch_file = None
            
    def on_closing(self):
            print("Closing GUI")
            # close the gui and exit the program
            self.destroy()
            rospy.signal_shutdown("GUI closed")
        
if __name__ == "__main__":
    root = NodeGUI()
    root.mainloop()
    rospy.signal_shutdown("GUI closed")
    