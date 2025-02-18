import pyautogui
import serial
import time
import numpy as np
import re

# Bluetooth Settings
bt_port = 'COM7'  # Replace with your COM port
baud_rate = 115200  # Match the baud rate in the ESP32 code

# Screen Settings
width, height = pyautogui.size()
pyautogui.moveTo(width // 2, height // 2)

# Smoothing Settings
prev_conv_x, prev_conv_y = width // 2, height // 2
alpha = 0.8  # Smoothing factor

# MPU Initial Values
mpu_initial_values = []

# Smoothing Function
def smooth_positions(prev_x, prev_y, new_x, new_y, alpha):
    smooth_x = alpha * prev_x + (1 - alpha) * new_x
    smooth_y = alpha * prev_y + (1 - alpha) * new_y
    return smooth_x, smooth_y

bt_serial = None
try:
    # Establish Bluetooth connection with a slightly longer timeout
    bt_serial = serial.Serial(bt_port, baud_rate, timeout=5)
    print(f"Connected to {bt_port}")

    while True:
        # Clear serial buffer to avoid old data causing lag
        bt_serial.reset_input_buffer()
        mpu_data = ''
        mpu_data_list = []

        # Read multiple lines to avoid missing data
        while bt_serial.in_waiting > 0:
            mpu_data = bt_serial.readline().decode('utf-8').strip()
            print(f"Received: {mpu_data}")

            # Ensure that the mpu_data is not empty before processing
            if mpu_data:
                try:
                    # Extract numerical values from "X: 0.74 | Y: -4.08" format
                    matches = re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", mpu_data)

                    if len(matches) == 2:
                        x_value = float(matches[0])
                        y_value = float(matches[1])
                        mpu_data_list = [x_value, y_value]
                    else:
                        print(f"Invalid data format: {mpu_data}")
                        continue

                    # Store initial values to calculate relative movement
                    if not mpu_initial_values:
                        mpu_initial_values = mpu_data_list

                except ValueError as e:
                    print(f"Error processing sensor data: {e}")
                    time.sleep(0.5)
                    continue

                mpu_move_coords = [
                    mpu_data_list[0] - mpu_initial_values[0],
                    mpu_data_list[1] - mpu_initial_values[1],
                ]

                # Interpolate and convert MPU values to screen coordinates
                conv_x = int(np.interp(mpu_move_coords[0], (-10, 10), (0, width)))
                conv_y = int(np.interp(mpu_move_coords[1], (-10, 10), (0, height)))

                # Apply smoothing
                smoothed_x, smoothed_y = smooth_positions(prev_conv_x, prev_conv_y, conv_x, conv_y, alpha)
                prev_conv_x, prev_conv_y = smoothed_x, smoothed_y

                # Move the mouse smoothly
                pyautogui.moveTo(int(smoothed_x), int(smoothed_y), duration=0.03)  # Faster gliding
                print("Moving mouse to:", int(smoothed_x), int(smoothed_y))

        # Reduced sleep time for more responsive movement
        time.sleep(0.01)

except serial.SerialException as e:
    print(f"Error connecting to {bt_port}: {e}")

except KeyboardInterrupt:
    print("Program stopped.")

finally:
    if bt_serial and bt_serial.is_open:
        bt_serial.close()
        print("Connection closed.")
