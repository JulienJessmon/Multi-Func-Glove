import serial
import time

bt_port = 'COM3'  # Replace with ur COM port
baud_rate = 115200  # Match the baud rate in the ESP32 code

try:
    bt_serial = serial.Serial(bt_port, baud_rate, timeout=1)
    print(f"Connected to {bt_port}")

    while True:
        if bt_serial.in_waiting > 0:
            sensor_data = bt_serial.readline().decode('utf-8').strip()
            print(f"Received: {sensor_data}")

        time.sleep(1)

except serial.SerialException as e:
    print(f"Error connecting to {bt_port}: {e}")

except KeyboardInterrupt:
    print("Program stopped.")

finally:
    if bt_serial.is_open:
        bt_serial.close()
        print("Connection closed.")
