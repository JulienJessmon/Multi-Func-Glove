import pyautogui
import serial
import time

bt_port = 'COM4'  # Replace with your COM port
baud_rate = 115200  # Match the baud rate in the ESP32 code

def single_match(activation_value, flex_input, bit_value):
    if bit_value:
        if flex_input > activation_value:
            return False
    else:
        if flex_input < activation_value:
            return False
    return True


def full_match(activation_values, flex_inputs, flex_bit_array):
    for i in range(len(activation_values)):
        if not single_match(activation_values[i], flex_inputs[i], flex_bit_array[i]):
            return False

    return True

def switch_tab():
    time.sleep(0.1)  # Add a slight delay before pressing the keys
    pyautogui.keyDown('alt')
    pyautogui.press('tab')
    pyautogui.keyUp('alt')
    time.sleep(0.1)  # Add a slight delay after pressing the keys


def minimize_all():
    pyautogui.hotkey('win', 'm')

def minimize_current():
    pyautogui.hotkey('win', 'down')

def restore_current():
    pyautogui.hotkey('win', 'up')

def screenshot():
    pyautogui.hotkey('win', 'shift', 's')

def tile_left_right():
    pyautogui.hotkey('win', 'right')
    time.sleep(0.5)
    pyautogui.press('enter')

def copy():
    pyautogui.hotkey('ctrl', 'c')

def paste():
    pyautogui.hotkey('ctrl', 'v')

def cut():
    pyautogui.hotkey('ctrl', 'x')

def undo():
    pyautogui.hotkey('ctrl', 'z')

def redo():
    pyautogui.hotkey('ctrl', 'y')

def select_all():
    pyautogui.hotkey('ctrl', 'a')

def save():
    pyautogui.hotkey('ctrl', 's')

def print_file():
    pyautogui.hotkey('ctrl', 'p')

def open_file_explorer():
    pyautogui.hotkey('win', 'e')

def rename():
    pyautogui.press('f2')

def delete():
    pyautogui.press('delete')

def permanent_delete():
    pyautogui.hotkey('shift', 'delete')

def switch_windows():
    pyautogui.hotkey('alt', 'tab')

def minimize_all_windows():
    pyautogui.hotkey('win', 'm')

def close_window():
    pyautogui.hotkey('alt', 'f4')

def lock_screen():
    pyautogui.hotkey('win', 'l')

def open_new_tab():
    pyautogui.hotkey('ctrl', 't')

def close_tab():
    pyautogui.hotkey('ctrl', 'w')

def search():
    pyautogui.hotkey('ctrl', 'l')

def refresh():
    pyautogui.hotkey('ctrl', 'r')

def open_settings():
    pyautogui.hotkey('win', 'i')

def open_run():
    pyautogui.hotkey('win', 'r')

def main():
    # activation_values = [1400, 749, 1275]
    activation_values = [1400, 300, 710]
    flex_inputs = [0, 0, 0]
    bt_serial = None
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
                    flex_inputs = [int(i) for i in sensor_data.split(',')]

                    # Check if the data matches the required range
                    if full_match(activation_values, flex_inputs, [1, 1, 1]):
                        # minimize_current()
                        print("minimize_current")
                    elif full_match(activation_values, flex_inputs, [1, 1, 0]):
                        # switch_tab()
                        print("switch_tab")
                    elif full_match(activation_values, flex_inputs, [1, 0, 1]):
                        # switch_windows()
                        print("switch_windows")
                    elif full_match(activation_values, flex_inputs, [1, 0, 0]):
                        # minimize_all()
                        print("minimize_all")
                    elif full_match(activation_values, flex_inputs, [0, 1, 1]):
                        # restore_current()
                        print("restore_current")
                    elif full_match(activation_values, flex_inputs, [0, 1, 0]):
                        # tile_left_right()
                        print("tile_left_right()")
                    elif full_match(activation_values, flex_inputs, [0, 0, 1]):
                        # screenshot()
                        print("screenshot()")
                    elif full_match(activation_values, flex_inputs, [0, 0, 0]):
                        pass

                except ValueError as e:
                    print(f"Error processing sensor data: {e}")

            # Wait before the next iteration to avoid busy-waiting
            time.sleep(1)

    except serial.SerialException as e:
        print(f"Error connecting to {bt_port}: {e}")

    except KeyboardInterrupt:
        print("Program stopped.")

    finally:
        if bt_serial and bt_serial.is_open:
            bt_serial.close()
            print("Connection closed.")

main()

