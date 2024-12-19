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
        root.geometry("350x200")  # Slightly larger window

        ctk.set_appearance_mode("Light")  # Appearance mode: Light/Dark/System
        ctk.set_default_color_theme("blue")  # Default color theme

        # Title Label
        ctk.CTkLabel(root, text="Shaking Table Control", font=("Arial", 16)).grid(row=0, column=0, columnspan=2, pady=10)

        # Amplitude Input
        ctk.CTkLabel(root, text='Amplitude').grid(row=1, column=0, padx=10, pady=5, sticky='w')
        self.amplitude_entry = ctk.CTkEntry(root, placeholder_text="e.g., 0.1")
        self.amplitude_entry.grid(row=1, column=1, padx=10, pady=5)

        # Frequency Input
        ctk.CTkLabel(root, text='Frequency').grid(row=2, column=0, padx=10, pady=5, sticky='w')
        self.frequency_entry = ctk.CTkEntry(root, placeholder_text="e.g., 5.0")
        self.frequency_entry.grid(row=2, column=1, padx=10, pady=5)

        # Set Button
        ctk.CTkButton(root, text='Set Parameters', command=self.set_parameters).grid(row=3, column=0, columnspan=2, pady=10)

        # Preset Buttons
        ctk.CTkButton(root, text='Low (A=0.1, F=1)', command=lambda: self.set_preset(0.1, 1.0)).grid(row=4, column=0, padx=5, pady=5)
        ctk.CTkButton(root, text='Medium (A=0.5, F=5)', command=lambda: self.set_preset(0.5, 5.0)).grid(row=4, column=1, padx=5, pady=5)

        # Stop Button
        ctk.CTkButton(root, text='Stop', command=self.stop_table).grid(row=5, column=0, columnspan=2, pady=10)

        # Status Label
        self.status_label = ctk.CTkLabel(root, text='', fg_color=None)
        self.status_label.grid(row=6, column=0, columnspan=2, pady=5)

        # Log Display
        self.log_display = ctk.CTkTextbox(root, height=4, width=30)
        self.log_display.grid(row=7, column=0, columnspan=2, pady=10)

    def set_parameters(self):
        try:
            # Get values from entries
            amplitude = float(self.amplitude_entry.get())
            frequency = float(self.frequency_entry.get())

            # Validate input values
            if amplitude <= 0 or frequency <= 0:
                rospy.logerr("Amplitude and frequency must be positive numbers.")
                self.show_status("Error: Positive values required", "red")
                return

            # Publish to ROS topics
            self.amplitude_pub.publish(amplitude)
            self.frequency_pub.publish(frequency)

            rospy.loginfo("Amplitude: %.3f, Frequency: %.3f", amplitude, frequency)
            self.show_status("Parameters set successfully!", "green")
            self.log_to_gui(f"Amplitude: {amplitude}, Frequency: {frequency}")
        except ValueError:
            rospy.logerr("Invalid input! Please enter valid numbers for amplitude and frequency.")
            self.show_status("Error: Invalid input", "red")
        except rospy.ROSException as e:
            rospy.logerr(f"ROS error: {e}")
            self.show_status(f"ROS Error: {e}", "red")

    def set_preset(self, amplitude, frequency):
        """Set preset values in the GUI."""
        self.amplitude_entry.delete(0, "end")
        self.amplitude_entry.insert(0, str(amplitude))
        self.frequency_entry.delete(0, "end")
        self.frequency_entry.insert(0, str(frequency))
        rospy.loginfo(f"Preset applied: Amplitude={amplitude}, Frequency={frequency}")
        self.show_status(f"Preset applied: A={amplitude}, F={frequency}", "blue")

    def stop_table(self):
        """Stop the shaking table by setting amplitude and frequency to zero."""
        self.amplitude_pub.publish(0.0)
        self.frequency_pub.publish(0.0)
        rospy.loginfo("Shaking table stopped.")
        self.show_status("Shaking table stopped", "orange")
        self.log_to_gui("Shaking table stopped.")

    def show_status(self, message, color):
        """Update a status label in the GUI."""
        self.status_label.configure(text=message, fg=color)

    def log_to_gui(self, message):
        """Display log messages in the GUI."""
        self.log_display.insert("end", message + "\n")
        self.log_display.yview("end")  # Auto-scroll to the latest message

if __name__ == '__main__':
    try:
        # Create the GUI application
        root = ctk.CTk()
        gui = ShakingTableGUI(root)

        # Handle window close
        def on_closing():
            rospy.loginfo("Shutting down ROS node...")
            rospy.signal_shutdown("GUI closed")
            root.destroy()

        root.protocol("WM_DELETE_WINDOW", on_closing)
        root.mainloop()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Node Interrupted. Exiting GUI...")
