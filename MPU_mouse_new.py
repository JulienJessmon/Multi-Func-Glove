# MPU mouse movement code

import serial
from pynput.mouse import Controller

# Replace with your Bluetooth COM port
bluetooth_port = "COM4"
baud_rate = 115200

# Initialize mouse controller
mouse = Controller()

# Calibration variables
num_calibration_samples = 500  # Increased calibration samples
gx_sum = 0
gy_sum = 0
gz_sum = 0
gx_offset = 0
gy_offset = 0
gz_offset = 0

# Filtering variables
alpha = 0.1  # Increased smoothing factor
filtered_gx = 0
filtered_gy = 0
filtered_gz = 0

# Sensitivity factors
sensitivity_x = 0.05  # Reduced sensitivity
sensitivity_y = 0.05

# Dead zone threshold
dead_zone = 1.0  # Increased dead zone

def apply_filter(raw_value, prev_filtered_value):
    return alpha * raw_value + (1 - alpha) * prev_filtered_value

def calibrate_sensor(ser):
    global gx_sum, gy_sum, gz_sum, gx_offset, gy_offset, gz_offset
    print("Calibrating MPU6050...")

    for _ in range(num_calibration_samples):
        data = ser.readline().decode('utf-8', errors='ignore').strip()
        if data:
            try:
                parts = data.split(",")
                if len(parts) >= 6:
                    # Extract MPU6050 values
                    ax = float(parts[0].split(":")[1])
                    ay = float(parts[1].split(":")[1])
                    az = float(parts[2].split(":")[1])
                    gx = float(parts[3].split(":")[1])
                    gy = float(parts[4].split(":")[1])
                    gz = float(parts[5].split(":")[1])

                    # Sum up gyroscope values for calibration
                    gx_sum += gx
                    gy_sum += gy
                    gz_sum += gz

            except Exception as e:
                print("Error parsing data during calibration:", e)

    gx_offset = gx_sum / num_calibration_samples
    gy_offset = gy_sum / num_calibration_samples
    gz_offset = gz_sum / num_calibration_samples
    print(f"Calibration complete. gx_offset: {gx_offset}, gy_offset: {gy_offset}, gz_offset: {gz_offset}")

'''def calibrate_sensor(ser):
    global gx_sum, gy_sum, gz_sum, gx_offset, gy_offset, gz_offset
    print("Calibrating MPU6050...")

    for _ in range(num_calibration_samples):
        data = ser.readline().decode('utf-8', errors='ignore').strip()
        if data:
            print(data)
            try:
                parts = data.split(", ")
                if len(parts) == 6:
                    gx = float(parts[3].split(":")[1])
                    gy = float(parts[4].split(":")[1])
                    gz = float(parts[5].split(":")[1])

                    gx_sum += gx
                    gy_sum += gy
                    gz_sum += gz

            except Exception as e:
                print("Error parsing data during calibration:", e)

    gx_offset = gx_sum / num_calibration_samples
    gy_offset = gy_sum / num_calibration_samples
    gz_offset = gz_sum / num_calibration_samples
    #print(f"Calibration complete. gx_offset: {gx_offset}, gy_offset: {gy_offset}, gz_offset: {gz_offset}")'''

try:
    ser = serial.Serial(bluetooth_port, baud_rate, timeout=1)
    print("Connected to ESP32 Bluetooth Serial")

    # Perform calibration
    calibrate_sensor(ser)

    while True:
        data = ser.readline().decode('utf-8', errors='ignore').strip()
        if data:
            try:
                parts = data.split(",")  # Split by comma
                if len(parts) >= 6:  # Ensure there are at least 6 parts (MPU6050 values)
                    # Extract MPU6050 values
                    ax = float(parts[0].split(":")[1])
                    ay = float(parts[1].split(":")[1])
                    az = float(parts[2].split(":")[1])
                    gx = float(parts[3].split(":")[1])
                    gy = float(parts[4].split(":")[1])
                    gz = float(parts[5].split(":")[1])

                    # Apply calibration to gyroscope values
                    gx -= gx_offset
                    gy -= gy_offset
                    gz -= gz_offset

                    # Apply low-pass filter
                    filtered_gx = apply_filter(gx, filtered_gx)
                    filtered_gy = apply_filter(gy, filtered_gy)
                    filtered_gz = apply_filter(gz, filtered_gz)

                    # Calculate mouse movement (Reduced speed)
                    dx = (-filtered_gx * sensitivity_x) / 2
                    dy = (filtered_gz * sensitivity_y) / 2

                    # Apply damping effect for smoother movement
                    dx = dx * 0.5  # Increased damping
                    dy = dy * 0.5

                    # Apply dead zone
                    if abs(dx) < dead_zone:
                        dx = 0
                    if abs(dy) < dead_zone:
                        dy = 0

                    # Print debugging info
                    print(f"Mouse movement: dx={dx}, dy={dy}")

                    # Move the mouse
                    if dx != 0 or dy != 0:
                        mouse.move(dx, -dy)

            except Exception as e:
                print("Error parsing data:", e)

except KeyboardInterrupt:
    print("Program interrupted by user.")

except Exception as e:
    print("Error:", e)

finally:
    if 'ser' in locals():
        ser.close()
        print("Serial connection closed.")
