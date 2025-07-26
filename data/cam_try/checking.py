#!/usr/bin/env python3

import mvsdk

DevList = mvsdk.CameraEnumerateDevice()
print(f"Detected {len(DevList)} camera(s)")
for i, dev in enumerate(DevList):
    serial_number = dev.GetSn()           # Access the serial number
    friendly_name = dev.GetFriendlyName() # Access user-friendly name
    port_type = dev.GetPortType()
    print(f"{i}: {friendly_name} | Port Type: {port_type} | SN: {serial_number}")


