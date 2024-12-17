#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import customtkinter as ctk

class ShakingTableGUI:
    def __init__(self, root):
        # Initialize ROS node
        rospy.init_node('shaking_table_gui', anonymous=True)

        # ROS Publishers
        self.amplitude_pub = rospy.Publisher('/shaking_table/amplitude', Float64, queue_size=10)
        self.frequency_pub = rospy.Publisher('/shaking_table/frequency', Float64, queue_size=10)

        # GUI Setup
        root.title('Shaking Table GUI')
        root.geometry("300x150")  # Optional: Set window size

        ctk.set_appearance_mode("Light")  # Appearance mode: Light/Dark/System
        ctk.set_default_color_theme("blue")  # Default color theme

        # Amplitude Input
        ctk.CTkLabel(root, text='Amplitude').grid(row=0, column=0, padx=10, pady=5, sticky='w')
        self.amplitude_entry = ctk.CTkEntry(root, placeholder_text="e.g., 0.1")
        self.amplitude_entry.grid(row=0, column=1, padx=10, pady=5)

        # Frequency Input
        ctk.CTkLabel(root, text='Frequency').grid(row=1, column=0, padx=10, pady=5, sticky='w')
        self.frequency_entry = ctk.CTkEntry(root, placeholder_text="e.g., 5.0")
        self.frequency_entry.grid(row=1, column=1, padx=10, pady=5)

        # Set Button
        ctk.CTkButton(root, text='Set Parameters', command=self.set_parameters).grid(row=2, column=0, columnspan=2, pady=10)

    def set_parameters(self):
        try:
            # Get values from entries
            amplitude = float(self.amplitude_entry.get())
            frequency = float(self.frequency_entry.get())

            # Publish to ROS topics
            self.amplitude_pub.publish(amplitude)
            self.frequency_pub.publish(frequency)

            rospy.loginfo("Amplitude: %.3f, Frequency: %.3f", amplitude, frequency)
        except ValueError:
            rospy.logerr("Invalid input! Please enter valid numbers for amplitude and frequency.")
        except rospy.ROSException as e:
            rospy.logerr(f"ROS error: {e}")

if __name__ == '__main__':
    try:
        # Create the GUI application
        root = ctk.CTk()
        gui = ShakingTableGUI(root)
        root.mainloop()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Node Interrupted. Exiting GUI...")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        