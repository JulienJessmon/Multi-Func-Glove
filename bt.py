import pyautogui
import serial
import time

bt_port = 'COM4'  # Replace with your COM port
baud_rate = 115200  # Match the baud rate in the ESP32 code


def is_match(sensor_data_list, required_range):
    if ((sensor_data_list[0] in required_range[0]) and
            # (sensor_data_list[1] in required_range[1]) and
            # (sensor_data_list[2] in required_range[2]) and
            # (sensor_data_list[3] in required_range[3]) and
            (sensor_data_list[1] in required_range[1])):
        return True
    else:
        return False


def switch_tab():
    pyautogui.keyDown('alt')
    pyautogui.press('tab')
    pyautogui.keyUp('alt')


def minimize_all():
    pyautogui.keyDown('win')
    pyautogui.press('m')
    pyautogui.keyUp('win')


def minimize_current():
    pyautogui.keyDown('win')
    pyautogui.press('down')
    pyautogui.keyUp('win')


def restore_current():
    pyautogui.keyDown('win')
    pyautogui.press('up')
    pyautogui.keyUp('win')


def screenshot():
    pyautogui.keyDown('win')
    pyautogui.keyDown('shift')
    pyautogui.press('s')
    pyautogui.keyUp('shift')
    pyautogui.keyUp('win')


def tile_left_right():
    pyautogui.keyDown('win')
    pyautogui.press('right')
    pyautogui.keyUp('win')

    time.sleep(0.5)

    pyautogui.press('enter')


try:
    bt_serial = serial.Serial(bt_port, baud_rate, timeout=3)
    print(f"Connected to {bt_port}")

    while True:
        sensor_data = ''

        # Check if data is available
        if bt_serial.in_waiting > 0:
            sensor_data = bt_serial.readline().decode('utf-8').strip()
            print(f"Received: {sensor_data}")

        # Ensure that the sensor_data is not empty before processing
        if sensor_data:
            try:
                # Split the received data into a list and convert to integers
                sensor_data_list = [int(i) for i in sensor_data.split(',')]

                # Check if the data matches the required range
                if is_match(sensor_data_list, [range(0, 800), range(0, 1230)]):
                    minimize_all()
                elif is_match(sensor_data_list, [range(0, 800), range(0,4096)]):
                    tile_left_right()
                elif is_match(sensor_data_list, [range(0, 4096), range(0, 1230)]):
                    switch_tab()




            except ValueError as e:
                print(f"Error processing sensor data: {e}")

        # Wait before the next iteration to avoid busy-waiting
        time.sleep(1)

except serial.SerialException as e:
    print(f"Error connecting to {bt_port}: {e}")

except KeyboardInterrupt:
    print("Program stopped.")

finally:
    if bt_serial.is_open:
        bt_serial.close()
        print("Connection closed.")
