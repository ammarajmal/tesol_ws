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
import rosgraph
import rospy
import rospkg
import roslaunch
import message_filters
from fiducial_msgs.msg import FiducialTransformArray
from scipy.signal import savgol_filter

themes = {'blue': ("#3B8ED0", "#1F6AA5", "#1f82d1"),
          'green': ("#2CC985", "#2FA572"),
          'dark-blue': ("#3a7ebf", "#1f538d"),
          'red': ("#fa5f5a", "#ba3732")}

COLOR_SELECT = list(themes.keys())[0]
ctk.set_appearance_mode("System")
ctk.set_default_color_theme(COLOR_SELECT)

class NodeGUI(ctk.CTk):
    def __init__(self, *args, **kwargs):
        super(NodeGUI, self).__init__(*args, **kwargs)
        self.title("Displacement Measurement System")
        self.geometry("1200x580")
        self.resizable(1, 1)
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.is_data_collection_active = False
        self.collected_data = []
        
        self.experiment_name = 'Cam3Sin'
        self.file_name = 'Cam'
        self.experiment_dur = 10  # seconds
        self.dir_name = 'UCer'
        self.exp_name_var = tk.StringVar(self, self.experiment_name)
        self.exp_dur_var = tk.StringVar(self, self.experiment_dur)

        self.create_widgets()
    
    def create_widgets(self):
        self.create_middle_second_frame()
    
    def create_middle_second_frame(self):
        self.middle_second_frame = tk.Frame(self, bg=themes[COLOR_SELECT][0])
        self.middle_second_frame.place(relx=0.5, rely=0, relwidth=0.25, relheight=1)
        self.create_middle_second_center_frame()
    
    def create_middle_second_center_frame(self):
        self.middle_second_center_frame = ctk.CTkFrame(self.middle_second_frame)
        self.middle_second_center_frame.place(relx=0.5, rely=0.2, relwidth=0.9, relheight=0.3, anchor='n')
        self.create_middle_second_center_frame_widgets()
    
    def create_middle_second_center_frame_widgets(self):
        self.middle_second_center_record_label = ctk.CTkLabel(self.middle_second_center_frame, text='RECORD DATA')
        self.middle_second_center_record_label.place(relx=0.5, rely=0.05, anchor='n')
        self.middle_second_center_exp_label = ctk.CTkLabel(self.middle_second_center_frame, text='Experiment Name: ')
        self.middle_second_center_exp_label.place(relx=0.1, rely=0.2)
        self.middle_second_center_exp_entry = ctk.CTkEntry(self.middle_second_center_frame, textvariable=self.exp_name_var)
        self.middle_second_center_exp_entry.place(relx=0.7, rely=0.2, anchor='n', relwidth=0.5)
        self.middle_second_center_dur_label = ctk.CTkLabel(self.middle_second_center_frame, text='Duration (s):')
        self.middle_second_center_dur_label.place(relx=0.1, rely=0.4)
        self.middle_second_center_dur_entry = ctk.CTkEntry(self.middle_second_center_frame, textvariable=self.exp_dur_var)
        self.middle_second_center_dur_entry.place(relx=0.7, rely=0.4, anchor='n', relwidth=0.3)
        self.middle_second_center_recall_button = ctk.CTkButton(self.middle_second_center_frame, text='RECORD', command=self.recall_data)
        self.middle_second_center_recall_button.place(relx=0.5, rely=0.7, anchor='n')
    
    def recall_data(self):
        print(f'Starting recording for {self.exp_name_var.get()}')
        self.is_data_collection_active = True
        self.sub = rospy.Subscriber('/sony_cam1/aruco_detect_node/fiducial_transforms', FiducialTransformArray, self.record_data)
        rospy.Timer(rospy.Duration(int(self.exp_dur_var.get())), self.stop_data_collection, oneshot=True)
    
    def record_data(self, msg):
        if not self.is_data_collection_active:
            return
        timestamp = msg.header.stamp.to_sec()
        for transform in msg.transforms:
            self.collected_data.append([timestamp, transform.fiducial_id, transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
    
    def stop_data_collection(self, event):
        self.is_data_collection_active = False
        self.sub.unregister()
        self.save_to_csv()
    
    def save_to_csv(self):
        if not self.collected_data:
            print("No data to save.")
            return
        df = pd.DataFrame(self.collected_data, columns=['Time (s)', 'Fiducial ID', 'X', 'Y', 'Z'])
        df.interpolate(method='linear', limit_direction='forward', inplace=True)
        file_name = f'{self.exp_name_var.get()}_{self.exp_dur_var.get()}s.csv'
        df.to_csv(file_name, index=False)
        print(f'Data saved to {file_name}')
    
    def on_closing(self):
        print("Closing application...")
        self.is_data_collection_active = False
        self.destroy()

if __name__ == "__main__":
    rospy.init_node('fin_gui_NUC3', anonymous=False)
    app = NodeGUI()
    app.mainloop()
