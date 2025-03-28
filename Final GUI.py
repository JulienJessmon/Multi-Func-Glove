# GUI with only flex sensor values read

import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QFrame, QScrollArea, QPushButton,
                             QDialog, QLineEdit, QFormLayout, QMessageBox, QComboBox, QMessageBox)
from PyQt5.QtGui import QPixmap, QMovie
from PyQt5.QtCore import Qt, QSize, QThread, pyqtSignal
import pyautogui
import serial
import time

bt_port = 'COM6'  # Replace with your COM port
baud_rate = 115200  # Match the baud rate in the ESP32 code
gesture_cooldown = 3

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

def switch_tab():
    time.sleep(0.1)  # Add a slight delay before pressing the keys
    pyautogui.keyDown('alt')
    pyautogui.press('tab')
    pyautogui.keyUp('alt')
    time.sleep(0.1)  # Add a slight delay after pressing the keys

def map_to_func(called_func):
    if called_func == "Minimize Current Window":
        minimize_current()
    elif called_func == "Switch Tab":
        switch_tab()
    elif called_func == "Switch Windows":
        switch_windows()
    elif called_func == "Minimize All Windows":
        minimize_all_windows()
    elif called_func == "Restore Current Window":
        restore_current()
    elif called_func == "Tile Left/Right":
        tile_left_right()
    elif called_func == "Screenshot":
        screenshot()

def single_match(activation_value, flex_input, bit_value, is_inverse=False):
    if bit_value:
        if is_inverse:
            # For inverse sensors, check if flex_input is less than activation_value
            if flex_input < activation_value:
                return False
        else:
            # For normal sensors, check if flex_input is greater than activation_value
            if flex_input > activation_value:
                return False
    else:
        if is_inverse:
            # For inverse sensors, check if flex_input is greater than activation_value
            if flex_input > activation_value:
                return False
        else:
            # For normal sensors, check if flex_input is less than activation_value
            if flex_input < activation_value:
                return False
    return True

def full_match(activation_values, flex_inputs, flex_bit_array):
    for i in range(len(activation_values)):
        # Check if the current sensor is the second one (index 1)
        is_inverse = (i == 1)  # Set is_inverse=True only for the second sensor
        if not single_match(activation_values[i], flex_inputs[i], flex_bit_array[i], is_inverse):
            return False
    return True

class CustomGesturesDialog(QDialog):
    functions_updated = pyqtSignal(list)  # New signal to emit when functions are updated
    def __init__(self, activation_values, func_list, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Custom Gestures")
        self.setModal(True)
        self.activation_values = activation_values
        self.functions = ["Minimize Current Window", "Switch Tab", "Switch Windows", "Minimize All Windows", "Restore Current Window", "Tile Left/Right", "Screenshot"]
        self.current_functions = func_list

        # Create input fields for each gesture
        self.form_layout = QFormLayout()
        self.combo_boxes = []
        for i in range(6):  # 6 gestures
            combo_box = QComboBox()
            combo_box.addItems(self.functions)
            combo_box.setCurrentText(self.current_functions[i % len(self.functions)])
            self.combo_boxes.append(combo_box)
            self.form_layout.addRow(f"Gesture {i + 1}:", combo_box)

        # Save button
        self.save_button = QPushButton("Save")
        self.save_button.clicked.connect(self.save_values)

        # Rotate Functions button
        self.rotate_button = QPushButton("Rotate Functions")
        self.rotate_button.clicked.connect(lambda: self.rotate_functions(parent, func_list))

        # Layout
        layout = QVBoxLayout()
        layout.addLayout(self.form_layout)
        layout.addWidget(self.save_button)
        layout.addWidget(self.rotate_button)
        self.setLayout(layout)

    def save_values(self):
        try:
            for i, combo_box in enumerate(self.combo_boxes):
                self.current_functions[i] = combo_box.currentText()
            self.functions_updated.emit(self.current_functions)  # Emit the updated functions
            QMessageBox.information(self, "Success", "Custom gestures saved successfully!")
            self.close()
        except Exception as e:
            QMessageBox.warning(self, "Error", f"An error occurred: {e}")

    def rotate_functions(self, main_window, func_list):
        # Rotate the functions to the next gesture
        self.current_functions = self.current_functions[-1:] + self.current_functions[:-1]
        for i, combo_box in enumerate(self.combo_boxes):
            combo_box.setCurrentText(self.current_functions[i])
        self.functions_updated.emit(self.current_functions)
        main_window.update_gesture_mappings(self.current_functions)
        main_window.current_functions = self.current_functions
        # No need to update thread separately since it now references main_window.current_functions


class BluetoothThread(QThread):
    update_status_signal = pyqtSignal(str)
    update_action_log_signal = pyqtSignal(str)

    def __init__(self, activation_values, main_window):
        super().__init__()
        self.activation_values = activation_values
        self.main_window = main_window  # Store reference to main window
        self.running = True
        self.last_gesture_time = 0
        self.gesture_cooldown = 3

    @property
    def current_functions(self):
        # Always get the current functions from the main window
        return self.main_window.current_functions

    def run(self):
        bt_serial = None
        try:
            bt_serial = serial.Serial(bt_port, baud_rate, timeout=3)
            self.update_status_signal.emit("Connected")
            print("Connected")
            while self.running:
                sensor_data = ""

                if bt_serial.in_waiting > 0:
                    sensor_data = bt_serial.readline().decode('utf-8').strip()
                    print(f"Received: {sensor_data}")

                if sensor_data:
                    try:
                        data_pairs = sensor_data.split(',')
                        sensor_values = {}

                        # Parse key-value pairs
                        for pair in data_pairs:
                            if ':' in pair:
                                key, value = pair.split(':')
                                sensor_values[key] = float(value)

                        # Process flex sensor data
                        flex_inputs = []
                        for i in range(1, 4):  # F1 to F3
                            key = f'F{i}'
                            if key in sensor_values:
                                flex_inputs.append(int(sensor_values[key]))  # Convert to int for flex sensors
                        print(f"Flex Inputs: {flex_inputs}")

                        # Check if cooldown has passed
                        current_time = time.time()
                        if current_time - self.last_gesture_time >= self.gesture_cooldown:
                            # Process gestures only if cooldown has passed
                            if full_match(self.activation_values, flex_inputs, [1, 0, 0]):
                                called_func = self.current_functions[0]
                                map_to_func(called_func)
                                self.update_action_log_signal.emit(self.current_functions[0])
                                self.last_gesture_time = current_time
                                # In BluetoothThread.run():
                            elif full_match(self.activation_values, flex_inputs, [1, 1, 0]):
                                self.main_window.rotate_functions()  # This is now thread-safe
                                self.update_action_log_signal.emit("Functions rotated!")
                                self.last_gesture_time = current_time
                            elif full_match(self.activation_values, flex_inputs, [0, 1, 0]):
                                self.update_action_log_signal.emit(self.current_functions[1])
                                self.last_gesture_time = current_time
                            """elif full_match(self.activation_values, flex_inputs, [1, 1, 1]):
                                self.update_action_log_signal.emit(self.current_functions[1])
                                self.last_gesture_time = current_time
                            elif full_match(self.activation_values, flex_inputs, [1, 0, 1]):
                                self.update_action_log_signal.emit(self.current_functions[2])
                                self.last_gesture_time = current_time
                            elif full_match(self.activation_values, flex_inputs, [0, 0, 1]):
                                self.update_action_log_signal.emit(self.current_functions[3])
                                self.last_gesture_time = current_time
                            elif full_match(self.activation_values, flex_inputs, [0, 1, 1]):
                                self.update_action_log_signal.emit(self.current_functions[4])
                                self.last_gesture_time = current_time"""


                    except ValueError as e:
                        print(f"Error processing sensor data: {e}")

                time.sleep(0.02)  # Small delay to avoid overwhelming the CPU

        except serial.SerialException as e:
            print(f"Error connecting to {bt_port}: {e}")
            self.update_status_signal.emit("Not Connected")

        finally:
            if bt_serial and bt_serial.is_open:
                bt_serial.close()
                print("Connection closed.")

    def stop(self):
        self.running = False
        self.quit()
        self.wait()



from PyQt5.QtWidgets import QFrame, QLabel, QVBoxLayout

from PyQt5.QtWidgets import QFrame, QLabel, QVBoxLayout, QPushButton

from PyQt5.QtWidgets import QFrame, QLabel, QVBoxLayout, QPushButton

class MainWindow(QMainWindow):

    rotate_functions_signal = pyqtSignal()
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Glove Interface")
        self.setGeometry(100, 100, 800, 600)  # Increased width to accommodate the side frame

        self.rotate_functions_signal.connect(self._rotate_functions)
        # Declare all widgets
        self.central_widget = QWidget()
        self.central_widget.setStyleSheet("background-color: #1b1c2a;"
                                          "font-family: Arial;"
                                          "font-size: 18px;")
        self.main_layout = QHBoxLayout(self.central_widget)

        # Left frame and its widgets
        self.left_frame = QFrame()
        self.left_frame.setStyleSheet("background-color: #2a2b41; border-radius: 10px;")
        self.left_layout = QVBoxLayout(self.left_frame)

        # Top frame inside left frame
        self.top_frame = QFrame()
        self.top_layout = QVBoxLayout(self.top_frame)

        # Image label
        self.image_label = QLabel()
        self.bluetooth_pixmap = QPixmap("bluetooth.png").scaled(25, 25,
                                                                Qt.KeepAspectRatio)  # Replace with your image path
        self.image_label.setPixmap(self.bluetooth_pixmap)
        self.image_label.setAlignment(Qt.AlignCenter)

        # Bluetooth text
        self.bluetooth_text = QLabel("Bluetooth")
        self.bluetooth_text.setStyleSheet("color: white; font-size: 18px;")
        self.bluetooth_text.setAlignment(Qt.AlignCenter)

        # Connecting text and loading GIF
        self.connecting_layout = QHBoxLayout()  # Layout for GIF and text
        self.connecting_text = QLabel("Connecting...")
        self.connecting_text.setStyleSheet("color: white; font-size: 14px;")

        # Loading GIF
        self.loading_gif = QLabel()
        self.movie = QMovie("spinning loader.gif")  # Replace with your GIF path
        self.movie.setScaledSize(QSize(30, 30))  # Set the size of the GIF
        self.loading_gif.setMovie(self.movie)
        self.movie.start()  # Start the GIF animation

        # Add GIF and text to the layout
        self.connecting_layout.addWidget(self.loading_gif)
        self.connecting_layout.addWidget(self.connecting_text)
        self.connecting_layout.setAlignment(Qt.AlignCenter)

        # Retry Connection Button
        self.retry_button = QPushButton("Retry Connection")
        self.retry_button.setStyleSheet("""
            QPushButton {
                background-color: #1f84ce;
                border: none;
                border-radius: 15px;
                padding: 5px;
                color: white;
            }
            QPushButton:hover {
                background-color: #2a93e0;
            }
            QPushButton:pressed {
                background-color: #1a73b6;
            }
        """)
        self.retry_button.setFixedSize(150, 50)
        self.retry_button.clicked.connect(self.retry_connection)
        self.retry_button.setVisible(False)  # Initially hidden

        # Custom Gestures Button
        self.custom_gestures_button = QPushButton("Custom Gestures")
        self.custom_gestures_button.setStyleSheet("""
            QPushButton {
                background-color: #1f84ce;
                border: none;
                border-radius: 15px;
                padding: 5px;
                color: white;
            }
            QPushButton:hover {
                background-color: #2a93e0;
            }
            QPushButton:pressed {
                background-color: #1a73b6;
            }
        """)
        self.custom_gestures_button.setFixedSize(150, 50)
        self.custom_gestures_button.clicked.connect(self.open_custom_gestures_dialog)

        # Center buttons in a horizontal layout
        self.button_layout = QHBoxLayout()
        self.button_layout.addWidget(self.retry_button)
        self.button_layout.addWidget(self.custom_gestures_button)
        self.button_layout.setAlignment(Qt.AlignCenter)

        # Bottom frame inside left frame
        self.bottom_frame = QFrame()
        self.bottom_layout = QVBoxLayout(self.bottom_frame)

        # Action Log heading
        self.action_log_heading = QLabel("Action Log")
        self.action_log_heading.setStyleSheet("color: white; font-size: 18px; font-weight: bold;")
        self.action_log_heading.setAlignment(Qt.AlignCenter)

        # Scroll area for dummy frames
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_content = QWidget()
        self.scroll_layout = QVBoxLayout(self.scroll_content)
        self.scroll_layout.setAlignment(Qt.AlignTop)
        self.scroll_area.setStyleSheet("""
            QScrollArea {
                border: none;
                background: #2a2b41; /* Set a solid background color */
            }

            QScrollBar:vertical {
                background: #3a3b51; /* Match background to blend in */
                width: 8px;
                border-radius: 4px;
            }

            QScrollBar::handle:vertical {
                background: #888;
                min-height: 20px;
                border-radius: 4px;
            }

            QScrollBar::handle:vertical:hover {
                background: #aaa;
            }

            QScrollBar::handle:vertical:pressed {
                background: #666;
            }

            QScrollBar::add-line:vertical, 
            QScrollBar::sub-line:vertical, 
            QScrollBar::up-arrow:vertical, 
            QScrollBar::down-arrow:vertical {
                background: none;
                height: 0px;
            }

            /* Horizontal Scrollbar */
            QScrollBar:horizontal {
                background: #3a3b51;
                height: 8px;
                border-radius: 4px;
            }

            QScrollBar::handle:horizontal {
                background: #888;
                min-width: 20px;
                border-radius: 4px;
            }

            QScrollBar::handle:horizontal:hover {
                background: #aaa;
            }

            QScrollBar::handle:horizontal:pressed {
                background: #666;
            }

            QScrollBar::add-line:horizontal, 
            QScrollBar::sub-line:horizontal, 
            QScrollBar::left-arrow:horizontal, 
            QScrollBar::right-arrow:horizontal {
                background: none;
                width: 0px;
            }
        """)

        # Clear actions button
        self.clear_action_log_button = QPushButton("Clear")
        self.clear_action_log_button.setStyleSheet("""
            QPushButton {
                background-color: #1f84ce;
                border: none;
                border-radius: 15px;
                padding: 5px;
                color: white;
            }
            QPushButton:hover {
                background-color: #2a93e0;
            }
            QPushButton:pressed {
                background-color: #1a73b6;
            }
        """)
        self.clear_action_log_button.setFixedSize(100, 50)
        self.clear_action_log_button.clicked.connect(self.clear_action_log)

        # Right frame for gesture-function mappings and MPU6050 calibration
        self.right_frame = QFrame()
        self.right_frame.setStyleSheet("background-color: #1b1c2a; border: none;")  # Transparent background
        self.right_layout = QVBoxLayout(self.right_frame)
        self.right_layout.setSpacing(20)  # Add spacing between the two frames

        # Gesture-Function Mappings frame
        self.gesture_mappings_frame = QFrame()
        self.gesture_mappings_frame.setStyleSheet("""
            background-color: #2a2b41;
            border-radius: 10px;
            padding: 10px;
        """)
        self.gesture_mappings_layout = QVBoxLayout(self.gesture_mappings_frame)

        # Gesture-Function Mappings heading
        self.mappings_heading = QLabel("Gesture-Function Mappings")
        self.mappings_heading.setStyleSheet("color: white; font-size: 18px; font-weight: bold;")
        self.mappings_heading.setAlignment(Qt.AlignCenter)

        # Add heading to the gesture mappings frame
        self.gesture_mappings_layout.addWidget(self.mappings_heading)

        # Labels to display gesture-function mappings
        self.activation_values = [1600, 335, 100]  # Example activation values
        self.current_functions = ["Minimize Current Window", "Switch Tab", "Switch Windows", "Minimize All Windows",
                                  "Restore Current Window", "Tile Left/Right", "Screenshot"]

        self.gesture_labels = []
        for i in range(6):  # 6 gestures
            label = QLabel(f"Gesture {i + 1}: {self.current_functions[i]}")
            label.setStyleSheet("color: white; font-size: 16px;")
            label.setAlignment(Qt.AlignCenter)
            self.gesture_labels.append(label)
            self.gesture_mappings_layout.addWidget(label)

        # Add gesture mappings frame to the right layout
        self.right_layout.addWidget(self.gesture_mappings_frame)

        # MPU6050 Calibration frame
        self.mpu_calibration_frame = QFrame()
        self.mpu_calibration_frame.setStyleSheet("""
            background-color: #2a2b41;
            border-radius: 10px;
            padding: 10px;
        """)
        self.mpu_calibration_layout = QVBoxLayout(self.mpu_calibration_frame)

        # Label for MPU6050 calibration status
        self.calibration_label = QLabel("MPU6050 Calibrated!")
        self.calibration_label.setStyleSheet("color: white; font-size: 16px;")
        self.calibration_label.setAlignment(Qt.AlignCenter)

        # Calibrate button
        self.calibrate_button = QPushButton("Calibrate")
        self.calibrate_button.setStyleSheet("""
            QPushButton {
                background-color: #1f84ce;
                border: none;
                border-radius: 15px;
                padding: 5px;
                color: white;
            }
            QPushButton:hover {
                background-color: #2a93e0;
            }
            QPushButton:pressed {
                background-color: #1a73b6;
            }
        """)
        self.calibrate_button.setFixedSize(150, 50)
        self.calibrate_button.clicked.connect(self.calibrate_mpu6050)

        # Add label and button to the calibration frame
        self.mpu_calibration_layout.addWidget(self.calibration_label)
        self.mpu_calibration_layout.addWidget(self.calibrate_button)
        self.mpu_calibration_layout.setAlignment(Qt.AlignCenter)

        # Add calibration frame to the right layout
        self.right_layout.addWidget(self.mpu_calibration_frame)

        # Initialize UI
        self.initUI()

        self.bt_thread = BluetoothThread(self.activation_values, self)
        self.bt_thread.update_status_signal.connect(self.update_status)
        self.bt_thread.update_action_log_signal.connect(self.add_action_to_log)
        self.bt_thread.start()
    def funcRotate(self, func_list):
        func_list.append(func_list[0])
        func_list.pop(0)
    def initUI(self):
        # Set central widget
        self.setCentralWidget(self.central_widget)

        # Add left and right frames to main layout
        self.main_layout.addWidget(self.left_frame)
        self.main_layout.addWidget(self.right_frame)

        # Set stretch factors for left and right frames
        self.main_layout.setStretch(0, 2)  # Left frame takes 2/3 of the space
        self.main_layout.setStretch(1, 1)  # Right frame takes 1/3 of the space

        # Add top and bottom frames to left frame
        self.left_layout.addWidget(self.top_frame)
        self.left_layout.addWidget(self.action_log_heading)
        self.left_layout.addWidget(self.bottom_frame)
        self.left_layout.addWidget(self.clear_action_log_button)

        # Add widgets to top frame
        self.top_layout.addWidget(self.image_label)
        self.top_layout.addWidget(self.bluetooth_text)
        self.top_layout.addLayout(self.connecting_layout)  # Add the GIF and text layout
        self.top_layout.addLayout(self.button_layout)  # Add centered buttons

        # Add widgets to bottom frame
        self.bottom_layout.addWidget(self.scroll_area)

        # Add dummy frames to scroll area
        #for i in range(1):
           #self.add_action_to_log("Switch Tab")

        self.scroll_area.setWidget(self.scroll_content)

        # Apply styles
        self.left_frame.setStyleSheet("background-color: #2a2b41; border-radius: 10px;")
        self.top_frame.setStyleSheet("background-color: #3a3b51; border-radius: 10px;")
        self.bottom_frame.setStyleSheet("background-color: #3a3b51; border-radius: 10px;")
        self.right_frame.setStyleSheet("background-color: #1b1c2a; border: none;")  # Transparent background

    def rotate_functions(self):
        """Thread-safe way to request a rotation"""
        self.rotate_functions_signal.emit()

    def _rotate_functions(self):
        """Actual rotation implementation (runs in main thread)"""
        # Rotate the functions
        self.current_functions = self.current_functions[-1:] + self.current_functions[:-1]
        self.update_gesture_mappings(self.current_functions)

    def update_gesture_mappings(self, func_list):
        """Update the labels in the right frame with the current gesture-function mappings."""
        for i, label in enumerate(self.gesture_labels):
            label.setText(f"Gesture {i + 1}: {func_list[i]}")

    def update_status(self, message):
        self.connecting_text.setText(message)
        # Show retry button only if not connected
        self.retry_button.setVisible(message == "Not Connected")

        # Show or hide the loading spinner based on the connection status
        if message == "Not Connected":
            self.loading_gif.setVisible(False)  # Hide the loading spinner
        else:
            self.loading_gif.setVisible(True)  # Show the loading spinner

    def add_action_to_log(self, action):
        log_label = QLabel(action)
        log_label.setStyleSheet(
            "background-color: #4a4b61; border-radius: 5px; margin: 5px; padding: 5px; color: white;")
        log_label.setFixedHeight(50)
        log_label.setAlignment(Qt.AlignCenter)
        self.scroll_layout.addWidget(log_label)

    def clear_action_log(self):
        while self.scroll_layout.count():
            item = self.scroll_layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()

    def retry_connection(self):
        """Restart the Bluetooth connection thread."""
        # Hide the loading spinner and show "Connecting..." text
        self.loading_gif.setVisible(True)
        self.retry_button.setVisible(False)
        self.connecting_text.setText("Connecting...")

        # Stop the existing thread if it's running
        if self.bt_thread.isRunning():
            self.bt_thread.stop()

        # Start a new thread
        self.bt_thread = BluetoothThread(self.activation_values, self)
        self.bt_thread.update_status_signal.connect(self.update_status)
        self.bt_thread.update_action_log_signal.connect(self.add_action_to_log)
        self.bt_thread.start()

    def open_custom_gestures_dialog(self):
        """Open a dialog to set custom gestures."""
        dialog = CustomGesturesDialog(self.activation_values, self.current_functions, self)
        dialog.functions_updated.connect(self.update_gesture_mappings)
        dialog.exec_()  # Don't need to check for Accepted since signal handles updates

    def calibrate_mpu6050(self):
        """Handle the calibration of the MPU6050."""
        # Placeholder for MPU6050 calibration logic
        QMessageBox.information(self, "Calibration", "MPU6050 calibration started!")
        # Simulate calibration completion
        self.calibration_label.setText("MPU6050 Calibrated!")

    def closeEvent(self, event):
        self.bt_thread.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
