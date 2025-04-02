import cv2
import mediapipe as mp
import time
import mouse
import numpy as np
import threading
import serial
from pynput.mouse import Controller
import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QFrame, QScrollArea, QPushButton,
                             QDialog, QLineEdit, QFormLayout, QMessageBox, QComboBox)
from PyQt5.QtGui import QPixmap, QMovie
from PyQt5.QtCore import Qt, QSize, QThread, pyqtSignal
import pyautogui
import warnings
warnings.filterwarnings("ignore", category=UserWarning, message="SymbolDatabase.GetPrototype.*")

# Constants
BT_PORT = 'COM6'  # Replace with your COM port
BAUD_RATE = 115200  # Match the baud rate in the ESP32 code
GESTURE_COOLDOWN = 3
CAM_W, CAM_H = 640, 480
FRAME_R = 100


class CameraHandTracking:
    def __init__(self, log_callback=None):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
            max_num_hands=1
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.cap = None
        self.pTime = 0
        self.running = False
        self.thread = None
        self.camera_initialized = False

        # Smoothing variables
        self.prev_conv_x, self.prev_conv_y = 0, 0
        self.alpha = 0.7  # Smoothing factor
        self.log_callback = log_callback

        # Thumb detection variables
        self.thumb_up_start_time = None
        self.thumb_up_duration_needed = 5  # seconds
        self.switch_to_flex_mode_callback = None
        self.thumb_debug = False  # Debug visuals disabled by default

        # Click cooldown variables
        self.last_click_time = 0
        self.click_cooldown = 2  # seconds

    def set_mode_switch_callback(self, callback):
        self.switch_to_flex_mode_callback = callback

    def landmark_distance(self, landmark1, landmark2):
        """Calculate Euclidean distance between two landmarks"""
        return ((landmark1.x - landmark2.x) ** 2 + (landmark1.y - landmark2.y) ** 2) ** 0.5

    def initialize_camera(self):
        """Initialize the camera with multiple attempts"""
        max_attempts = 3
        for attempt in range(max_attempts):
            self.cap = cv2.VideoCapture(0)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
                self.camera_initialized = True
                return True
            time.sleep(1)  # Wait before retrying
        return False

    def start(self):
        if not self.camera_initialized and not self.initialize_camera():
            print("Failed to initialize camera")
            return

        self.running = True
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive() and threading.current_thread() != self.thread:
            self.thread.join()
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        self.camera_initialized = False

    def smooth_positions(self, prev_x, prev_y, new_x, new_y, alpha):
        smoothed_x = alpha * prev_x + (1 - alpha) * new_x
        smoothed_y = alpha * prev_y + (1 - alpha) * new_y
        return smoothed_x, smoothed_y

    def run(self):
        while self.running:
            ret, img = self.cap.read()
            if not ret:
                print("Failed to grab frame - retrying...")
                if not self.initialize_camera():
                    time.sleep(0.1)
                    continue
                else:
                    ret, img = self.cap.read()
                    if not ret:
                        continue

            img = cv2.flip(img, 1)
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = self.hands.process(img_rgb)

            cv2.rectangle(img, (FRAME_R, FRAME_R), (CAM_W - FRAME_R, CAM_H - FRAME_R), (255, 0, 255), 2)

            current_time = time.time()  # Get current time once per frame

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(img, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                    # Get finger landmarks
                    thumb_tip = hand_landmarks.landmark[4]
                    thumb_ip = hand_landmarks.landmark[3]
                    thumb_mcp = hand_landmarks.landmark[2]

                    index_tip = hand_landmarks.landmark[8]
                    index_pip = hand_landmarks.landmark[6]

                    middle_tip = hand_landmarks.landmark[12]
                    middle_pip = hand_landmarks.landmark[10]

                    ring_tip = hand_landmarks.landmark[16]
                    pinky_tip = hand_landmarks.landmark[20]

                    # Calculate distances for thumb detection
                    tip_ip_dist = self.landmark_distance(thumb_tip, thumb_ip)
                    tip_mcp_dist = self.landmark_distance(thumb_tip, thumb_mcp)

                    # Improved thumb detection
                    thumb_up = (thumb_tip.y < thumb_ip.y and  # Tip above IP joint
                                thumb_tip.y < thumb_mcp.y and  # Tip above MCP joint
                                tip_ip_dist > 0.05 and  # Tip sufficiently far from IP
                                tip_mcp_dist > 0.1)  # Tip sufficiently far from MCP

                    # Thumb-up gesture timing
                    if thumb_up:
                        if self.thumb_up_start_time is None:
                            self.thumb_up_start_time = current_time
                        else:
                            current_duration = current_time - self.thumb_up_start_time
                            if current_duration >= self.thumb_up_duration_needed:
                                if self.switch_to_flex_mode_callback:
                                    self.log_callback("Detected thumb-up for 5 seconds")
                                    self.switch_to_flex_mode_callback()
                                    self.thumb_up_start_time = None
                                    break  # Exit the loop to switch modes
                    else:
                        self.thumb_up_start_time = None

                    # Check which fingers are up
                    fingers_up = [
                        thumb_up,  # We use our improved thumb detection
                        index_tip.y < index_pip.y,  # Index finger
                        middle_tip.y < middle_pip.y,  # Middle finger
                        ring_tip.y < hand_landmarks.landmark[14].y,  # Ring finger
                        pinky_tip.y < hand_landmarks.landmark[18].y  # Pinky finger
                    ]

                    # Cursor movement mode (only index finger up)
                    if fingers_up[1] and not fingers_up[2] and not fingers_up[3] and not fingers_up[4]:
                        # Convert coordinates to screen resolution
                        conv_x = int(np.interp(index_tip.x * CAM_W, (FRAME_R, CAM_W - FRAME_R), (0, 1536)))
                        conv_y = int(np.interp(index_tip.y * CAM_H, (FRAME_R, CAM_H - FRAME_R), (0, 864)))

                        # Apply smoothing
                        smoothed_x, smoothed_y = self.smooth_positions(
                            self.prev_conv_x, self.prev_conv_y, conv_x, conv_y, self.alpha
                        )
                        self.prev_conv_x, self.prev_conv_y = smoothed_x, smoothed_y

                        # Move mouse
                        mouse.move(int(smoothed_x), int(smoothed_y))

                    # Left click (index and middle fingers up) with cooldown
                    if (fingers_up[1] and fingers_up[2] and not fingers_up[3] and not fingers_up[4] and
                            current_time - self.last_click_time >= self.click_cooldown):
                        if abs(index_tip.x - middle_tip.x) * CAM_W < 25:  # Fingers close together
                            mouse.click(button='left')
                            self.last_click_time = current_time
                            if self.log_callback:
                                self.log_callback("Left mouse click performed")

                    # Right click (index, middle and ring fingers up) with cooldown
                    if (fingers_up[1] and fingers_up[2] and fingers_up[3] and not fingers_up[4] and
                            current_time - self.last_click_time >= self.click_cooldown):
                        if (abs(index_tip.x - middle_tip.x) * CAM_W < 25 and
                                abs(middle_tip.x - ring_tip.x) * CAM_W < 25):
                            mouse.click(button='right')
                            self.last_click_time = current_time
                            if self.log_callback:
                                self.log_callback("Right mouse click performed")

                    # Visual feedback for thumb detection
                    if thumb_up and self.thumb_up_start_time:
                        duration = current_time - self.thumb_up_start_time
                        progress = min(1.0, duration / self.thumb_up_duration_needed)

                        # Draw progress bar
                        bar_width = 200
                        bar_height = 20
                        bar_x = CAM_W // 2 - bar_width // 2
                        bar_y = 50

                        # Background
                        cv2.rectangle(img, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height),
                                      (100, 100, 100), -1)
                        # Progress
                        cv2.rectangle(img, (bar_x, bar_y),
                                      (bar_x + int(bar_width * progress), bar_y + bar_height),
                                      (0, 255, 0), -1)
                        # Border
                        cv2.rectangle(img, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height),
                                      (255, 255, 255), 2)

                        # Text
                        cv2.putText(img, "Hold thumb up to switch mode", (bar_x, bar_y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # Calculate and display FPS
            cTime = time.time()
            fps = 1 / (cTime - self.pTime)
            self.pTime = cTime
            cv2.putText(img, f"FPS: {int(fps)}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

            # Display the image
            cv2.imshow("Camera Hand Tracking", img)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Clean up
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()


class FlexSensorControl:
    def __init__(self, log_callback=None):
        self.activation_values = [1200, 205, 100]
        self.current_functions = ["Minimize Current Window", "Switch Tab", "Switch Windows",
                                  "Minimize All Windows", "Restore Current Window", "Tile Left/Right", "Screenshot"]
        self.last_gesture_time = 0
        self.gesture_cooldown = GESTURE_COOLDOWN
        self.running = False
        self.thread = None
        self.log_callback = log_callback

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()

    def single_match(self, activation_value, flex_input, bit_value, is_inverse=False):
        if bit_value:
            if is_inverse:
                if flex_input < activation_value:
                    return False
            else:
                if flex_input > activation_value:
                    return False
        else:
            if is_inverse:
                if flex_input > activation_value:
                    return False
            else:
                if flex_input < activation_value:
                    return False
        return True

    def full_match(self, activation_values, flex_inputs, flex_bit_array):
        # Check if we have enough inputs to compare
        if len(flex_inputs) < len(activation_values):
            return False

        for i in range(len(activation_values)):
            is_inverse = (i == 1)  # Set is_inverse=True only for the second sensor
            if not self.single_match(activation_values[i], flex_inputs[i], flex_bit_array[i], is_inverse):
                return False
        return True

    def run(self):
        try:
            bt_serial = serial.Serial(BT_PORT, BAUD_RATE, timeout=3)
            print("Connected to flex sensors")
            if self.log_callback:
                self.log_callback("Connected to flex sensors")

            while self.running:
                sensor_data = ""
                if bt_serial.in_waiting > 0:
                    try:
                        sensor_data = bt_serial.readline().decode('utf-8').strip()
                    except UnicodeDecodeError:
                        continue  # Skip corrupted data

                if sensor_data:
                    try:
                        data_pairs = sensor_data.split(',')
                        sensor_values = {}
                        for pair in data_pairs:
                            if ':' in pair:
                                key, value = pair.split(':')
                                try:
                                    sensor_values[key] = float(value)
                                except ValueError:
                                    continue  # Skip invalid values

                        flex_inputs = []
                        for i in range(1, 4):  # F1 to F3
                            key = f'F{i}'
                            if key in sensor_values:
                                flex_inputs.append(int(sensor_values[key]))
                        print(flex_inputs)
                        # Only proceed if we got all 3 sensor readings
                        if len(flex_inputs) != 3:
                            continue

                        current_time = time.time()
                        if current_time - self.last_gesture_time >= self.gesture_cooldown:
                            try:
                                if self.full_match(self.activation_values, flex_inputs, [1, 0, 0]):
                                    self.map_to_func(self.current_functions[0])
                                    self.last_gesture_time = current_time
                                    if self.log_callback:
                                        self.log_callback(f"Gesture: {self.current_functions[0]}")
                                elif self.full_match(self.activation_values, flex_inputs, [1, 1, 0]):
                                    self.rotate_functions()
                                    self.last_gesture_time = current_time
                                    if self.log_callback:
                                        self.log_callback("Functions rotated")
                                elif self.full_match(self.activation_values, flex_inputs, [0, 1, 0]):
                                    self.map_to_func(self.current_functions[1])
                                    self.last_gesture_time = current_time
                                    if self.log_callback:
                                        self.log_callback(f"Gesture: {self.current_functions[1]}")
                            except Exception as e:
                                if self.log_callback:
                                    self.log_callback(f"Sensor error: {str(e)}")
                                continue

                    except ValueError as e:
                        if self.log_callback:
                            self.log_callback(f"Data processing error: {e}")
                        continue

                time.sleep(0.02)

        except serial.SerialException as e:
            if self.log_callback:
                self.log_callback(f"Serial connection error: {e}")
        except Exception as e:
            if self.log_callback:
                self.log_callback(f"Unexpected error: {e}")
        finally:
            if 'bt_serial' in locals() and bt_serial.is_open:
                bt_serial.close()

    def map_to_func(self, called_func):
        if called_func == "Minimize Current Window":
            pyautogui.hotkey('win', 'down')
        elif called_func == "Switch Tab":
            time.sleep(0.1)
            pyautogui.keyDown('alt')
            pyautogui.press('tab')
            pyautogui.keyUp('alt')
            time.sleep(0.1)
        elif called_func == "Switch Windows":
            pyautogui.hotkey('alt', 'tab')
        elif called_func == "Minimize All Windows":
            pyautogui.hotkey('win', 'm')
        elif called_func == "Restore Current Window":
            pyautogui.hotkey('win', 'up')
        elif called_func == "Tile Left/Right":
            pyautogui.hotkey('win', 'right')
            time.sleep(0.5)
            pyautogui.press('enter')
        elif called_func == "Screenshot":
            pyautogui.hotkey('win', 'shift', 's')

    def rotate_functions(self):
        self.current_functions = self.current_functions[-1:] + self.current_functions[:-1]


class MPUMouseControl:
    def __init__(self, log_callback=None):
        self.mouse = Controller()
        self.running = False
        self.thread = None

        # Calibration variables
        self.num_calibration_samples = 500
        self.gx_sum = 0
        self.gy_sum = 0
        self.gz_sum = 0
        self.gx_offset = 0
        self.gy_offset = 0
        self.gz_offset = 0

        # Filtering variables
        self.alpha = 0.1
        self.filtered_gx = 0
        self.filtered_gy = 0
        self.filtered_gz = 0

        # Sensitivity factors
        self.sensitivity_x = 0.05
        self.sensitivity_y = 0.05

        # Dead zone threshold
        self.dead_zone = 1.0
        self.log_callback = log_callback

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()

    def apply_filter(self, raw_value, prev_filtered_value):
        return self.alpha * raw_value + (1 - self.alpha) * prev_filtered_value

    def calibrate_sensor(self, ser):
        print("Calibrating MPU6050...")
        for _ in range(self.num_calibration_samples):
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            if data:
                try:
                    parts = data.split(",")
                    if len(parts) >= 6:
                        gx = float(parts[3].split(":")[1])
                        gy = float(parts[4].split(":")[1])
                        gz = float(parts[5].split(":")[1])

                        self.gx_sum += gx
                        self.gy_sum += gy
                        self.gz_sum += gz

                except Exception as e:
                    print("Error parsing data during calibration:", e)

        self.gx_offset = self.gx_sum / self.num_calibration_samples
        self.gy_offset = self.gy_sum / self.num_calibration_samples
        self.gz_offset = self.gz_sum / self.num_calibration_samples
        print(
            f"Calibration complete. gx_offset: {self.gx_offset}, gy_offset: {self.gy_offset}, gz_offset: {self.gz_offset}")

    def run(self):
        try:
            ser = serial.Serial(BT_PORT, BAUD_RATE, timeout=1)
            print("Connected to MPU6050")

            # Perform calibration
            self.calibrate_sensor(ser)

            while self.running:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    try:
                        parts = data.split(",")
                        if len(parts) >= 6:
                            gx = float(parts[3].split(":")[1])
                            gy = float(parts[4].split(":")[1])
                            gz = float(parts[5].split(":")[1])

                            # Apply calibration
                            gx -= self.gx_offset
                            gy -= self.gy_offset
                            gz -= self.gz_offset

                            # Apply filter
                            self.filtered_gx = self.apply_filter(gx, self.filtered_gx)
                            self.filtered_gy = self.apply_filter(gy, self.filtered_gy)
                            self.filtered_gz = self.apply_filter(gz, self.filtered_gz)

                            # Calculate mouse movement
                            dx = (-self.filtered_gx * self.sensitivity_x) / 2
                            dy = (self.filtered_gz * self.sensitivity_y) / 2

                            # Apply damping
                            dx = dx * 0.5
                            dy = dy * 0.5

                            # Apply dead zone
                            if abs(dx) < self.dead_zone:
                                dx = 0
                            if abs(dy) < self.dead_zone:
                                dy = 0

                            # Move mouse
                            if dx != 0 or dy != 0:
                                self.mouse.move(dx, -dy)

                    except Exception as e:
                        print("Error parsing data:", e)

        except KeyboardInterrupt:
            print("Program interrupted by user.")
        except Exception as e:
            print("Error:", e)
        finally:
            if 'ser' in locals():
                ser.close()


class CustomGesturesDialog(QDialog):
    functions_updated = pyqtSignal(list)

    def __init__(self, activation_values, func_list, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Custom Gestures")
        self.setModal(True)
        self.activation_values = activation_values
        self.functions = ["Minimize Current Window", "Switch Tab", "Switch Windows",
                          "Minimize All Windows", "Restore Current Window", "Tile Left/Right", "Screenshot"]
        self.current_functions = func_list

        self.form_layout = QFormLayout()
        self.combo_boxes = []
        for i in range(6):
            combo_box = QComboBox()
            combo_box.addItems(self.functions)
            combo_box.setCurrentText(self.current_functions[i % len(self.functions)])
            self.combo_boxes.append(combo_box)
            self.form_layout.addRow(f"Gesture {i + 1}:", combo_box)

        self.save_button = QPushButton("Save")
        self.save_button.clicked.connect(self.save_values)

        self.rotate_button = QPushButton("Rotate Functions")
        self.rotate_button.clicked.connect(lambda: self.rotate_functions(parent, func_list))

        layout = QVBoxLayout()
        layout.addLayout(self.form_layout)
        layout.addWidget(self.save_button)
        layout.addWidget(self.rotate_button)
        self.setLayout(layout)

    def save_values(self):
        try:
            for i, combo_box in enumerate(self.combo_boxes):
                self.current_functions[i] = combo_box.currentText()
            self.functions_updated.emit(self.current_functions)
            QMessageBox.information(self, "Success", "Custom gestures saved successfully!")
            self.close()
        except Exception as e:
            QMessageBox.warning(self, "Error", f"An error occurred: {e}")

    def rotate_functions(self, main_window, func_list):
        self.current_functions = self.current_functions[-1:] + self.current_functions[:-1]
        for i, combo_box in enumerate(self.combo_boxes):
            combo_box.setCurrentText(self.current_functions[i])
        self.functions_updated.emit(self.current_functions)
        main_window.update_gesture_mappings(self.current_functions)
        main_window.current_functions = self.current_functions


class MainWindow(QMainWindow):
    rotate_functions_signal = pyqtSignal()
    log_signal = pyqtSignal(str)
    def __init__(self):
        super().__init__()
        self.log_signal.connect(self.add_action_to_log)
        self.setWindowTitle("Unified Glove Control")
        self.setGeometry(100, 100, 800, 600)

        # Initialize control systems
        self.camera_control = CameraHandTracking(log_callback=self.log_signal.emit)
        self.flex_control = FlexSensorControl(log_callback=self.log_signal.emit)
        self.mpu_control = MPUMouseControl(log_callback=self.log_signal.emit)

        # Current mode (0: camera, 1: flex, 2: mpu)
        self.current_mode = 0

        # Setup UI
        self.initUI()

        # Start with camera mode
        self.switch_to_camera_mode()

    def initUI(self):
        self.central_widget = QWidget()
        self.central_widget.setStyleSheet("""
            background-color: #1b1c2a;
            font-family: Arial;
            font-size: 18px;
        """)
        self.main_layout = QHBoxLayout(self.central_widget)

        # Left frame
        self.left_frame = QFrame()
        self.left_frame.setStyleSheet("""
            background-color: #2a2b41;
            border-radius: 10px;
        """)
        self.left_layout = QVBoxLayout(self.left_frame)

        # Mode selection buttons
        self.mode_frame = QFrame()
        self.mode_layout = QVBoxLayout(self.mode_frame)

        self.camera_mode_btn = QPushButton("Camera Mode")
        self.camera_mode_btn.setStyleSheet(self.get_mode_button_style(True))
        self.camera_mode_btn.clicked.connect(self.switch_to_camera_mode)

        self.flex_mode_btn = QPushButton("Flex Sensor Mode")
        self.flex_mode_btn.setStyleSheet(self.get_mode_button_style(False))
        self.flex_mode_btn.clicked.connect(self.switch_to_flex_mode)

        self.mpu_mode_btn = QPushButton("MPU Mouse Mode")
        self.mpu_mode_btn.setStyleSheet(self.get_mode_button_style(False))
        self.mpu_mode_btn.clicked.connect(self.switch_to_mpu_mode)

        self.mode_layout.addWidget(self.camera_mode_btn)
        self.mode_layout.addWidget(self.flex_mode_btn)
        self.mode_layout.addWidget(self.mpu_mode_btn)

        # Status area
        self.status_frame = QFrame()
        self.status_layout = QVBoxLayout(self.status_frame)

        self.status_label = QLabel("Status: Active")
        self.status_label.setStyleSheet("color: white; font-size: 16px;")

        self.mode_label = QLabel("Current Mode: Camera Hand Tracking")
        self.mode_label.setStyleSheet("color: white; font-size: 16px;")

        self.status_layout.addWidget(self.status_label)
        self.status_layout.addWidget(self.mode_label)

        # Add to left layout
        self.left_layout.addWidget(self.mode_frame)
        self.left_layout.addWidget(self.status_frame)

        # Action Log area
        self.action_log_frame = QFrame()
        self.action_log_frame.setStyleSheet("""
            background-color: #2a2b41;
            border-radius: 10px;
        """)
        self.action_log_layout = QVBoxLayout(self.action_log_frame)

        # Action Log heading
        self.action_log_heading = QLabel("Action Log")
        self.action_log_heading.setStyleSheet("""
            color: white; 
            font-size: 18px; 
            font-weight: bold;
        """)
        self.action_log_heading.setAlignment(Qt.AlignCenter)

        # Scroll area for action log
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_content = QWidget()
        self.scroll_layout = QVBoxLayout(self.scroll_content)
        self.scroll_layout.setAlignment(Qt.AlignTop)
        self.scroll_area.setWidget(self.scroll_content)

        # Style the scroll area
        self.scroll_area.setStyleSheet("""
            QScrollArea {
                border: none;
                background: transparent;
            }
            QScrollBar:vertical {
                background: #3a3b51;
                width: 8px;
                border-radius: 4px;
            }
            QScrollBar::handle:vertical {
                background: #888;
                min-height: 20px;
                border-radius: 4px;
            }
        """)

        # Clear button
        self.clear_log_btn = QPushButton("Clear Log")
        self.clear_log_btn.setStyleSheet("""
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
        self.clear_log_btn.clicked.connect(self.clear_action_log)

        # Add widgets to action log frame
        self.action_log_layout.addWidget(self.action_log_heading)
        self.action_log_layout.addWidget(self.scroll_area)
        self.action_log_layout.addWidget(self.clear_log_btn)

        # Add action log frame to left layout (after status_frame)
        self.left_layout.addWidget(self.action_log_frame)

        # Right frame
        self.right_frame = QFrame()
        self.right_frame.setStyleSheet("""
            background-color: #2a2b41;
            border-radius: 10px;
        """)
        self.right_layout = QVBoxLayout(self.right_frame)

        # Gesture mappings (for flex mode)
        self.gesture_mappings_frame = QFrame()
        self.gesture_mappings_layout = QVBoxLayout(self.gesture_mappings_frame)

        self.mappings_heading = QLabel("Gesture-Function Mappings")
        self.mappings_heading.setStyleSheet("""
            color: white;
            font-size: 18px;
            font-weight: bold;
        """)
        self.mappings_heading.setAlignment(Qt.AlignCenter)

        self.gesture_mappings_layout.addWidget(self.mappings_heading)

        self.gesture_labels = []
        for i in range(6):
            label = QLabel(f"Gesture {i + 1}: {self.flex_control.current_functions[i]}")
            label.setStyleSheet("color: white; font-size: 16px;")
            label.setAlignment(Qt.AlignCenter)
            self.gesture_labels.append(label)
            self.gesture_mappings_layout.addWidget(label)

        self.custom_gestures_btn = QPushButton("Customize Gestures")
        self.custom_gestures_btn.setStyleSheet("""
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
        self.custom_gestures_btn.clicked.connect(self.open_custom_gestures_dialog)

        self.gesture_mappings_layout.addWidget(self.custom_gestures_btn)

        # MPU calibration (for MPU mode)
        self.mpu_calibration_frame = QFrame()
        self.mpu_calibration_layout = QVBoxLayout(self.mpu_calibration_frame)

        self.calibration_label = QLabel("MPU6050 Calibration")
        self.calibration_label.setStyleSheet("""
            color: white;
            font-size: 16px;
            font-weight: bold;
        """)
        self.calibration_label.setAlignment(Qt.AlignCenter)

        self.calibrate_btn = QPushButton("Calibrate")
        self.calibrate_btn.setStyleSheet("""
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
        self.calibrate_btn.clicked.connect(self.calibrate_mpu)

        self.mpu_calibration_layout.addWidget(self.calibration_label)
        self.mpu_calibration_layout.addWidget(self.calibrate_btn)

        # Add to right layout
        self.right_layout.addWidget(self.gesture_mappings_frame)
        self.right_layout.addWidget(self.mpu_calibration_frame)

        # Add frames to main layout
        self.main_layout.addWidget(self.left_frame)
        self.main_layout.addWidget(self.right_frame)

        self.setCentralWidget(self.central_widget)

    def add_action_to_log(self, message):
        """Add a message to the action log"""
        log_label = QLabel(message)
        log_label.setStyleSheet("""
            background-color: #3a3b51;
            border-radius: 5px;
            padding: 5px;
            margin: 2px;
            color: white;
        """)
        log_label.setWordWrap(True)
        self.scroll_layout.addWidget(log_label)

        # Scroll to bottom
        scroll_bar = self.scroll_area.verticalScrollBar()
        scroll_bar.setValue(scroll_bar.maximum())

    def clear_action_log(self):
        """Clear all items from the action log"""
        while self.scroll_layout.count():
            item = self.scroll_layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()

    def get_mode_button_style(self, is_active):
        base_style = """
            QPushButton {
                border: none;
                border-radius: 15px;
                padding: 10px;
                font-size: 16px;
        """
        if is_active:
            base_style += """
                background-color: #1f84ce;
                color: white;
                font-weight: bold;
            """
        else:
            base_style += """
                background-color: #3a3b51;
                color: #aaaaaa;
            """
        base_style += "}"
        return base_style

    def switch_to_camera_mode(self):
        self.stop_current_mode()
        self.current_mode = 0
        self.camera_control.set_mode_switch_callback(self.safe_switch_to_flex_mode)  # Changed to safe method
        self.camera_control.start()
        # Update UI
        self.update_ui_for_mode("Camera Hand Tracking")

    def safe_switch_to_flex_mode(self):
        # Use QTimer to schedule the mode switch in the main thread
        from PyQt5.QtCore import QTimer
        QTimer.singleShot(0, self.perform_flex_mode_switch)

    def perform_flex_mode_switch(self):
        """Perform the actual mode switch in the main thread"""
        self.stop_current_mode()
        self.current_mode = 1
        self.flex_control.start()
        self.update_ui_for_mode("Flex Sensor Control")
        self.log_signal.emit("Switched to Flex Mode (thumb gesture)")

    def update_ui_for_mode(self, mode_name):
        """Update UI elements for the current mode"""
        self.camera_mode_btn.setStyleSheet(self.get_mode_button_style(self.current_mode == 0))
        self.flex_mode_btn.setStyleSheet(self.get_mode_button_style(self.current_mode == 1))
        self.mpu_mode_btn.setStyleSheet(self.get_mode_button_style(self.current_mode == 2))
        self.mode_label.setText(f"Current Mode: {mode_name}")
    def switch_to_flex_mode(self):
        self.stop_current_mode()
        self.current_mode = 1
        self.flex_control.start()

        # Update UI
        self.camera_mode_btn.setStyleSheet(self.get_mode_button_style(False))
        self.flex_mode_btn.setStyleSheet(self.get_mode_button_style(True))
        self.mpu_mode_btn.setStyleSheet(self.get_mode_button_style(False))
        self.mode_label.setText("Current Mode: Flex Sensor Control")

    def switch_to_mpu_mode(self):
        self.stop_current_mode()
        self.current_mode = 2
        self.mpu_control.start()

        # Update UI
        self.camera_mode_btn.setStyleSheet(self.get_mode_button_style(False))
        self.flex_mode_btn.setStyleSheet(self.get_mode_button_style(False))
        self.mpu_mode_btn.setStyleSheet(self.get_mode_button_style(True))
        self.mode_label.setText("Current Mode: MPU Mouse Control")

    def stop_current_mode(self):
        if self.current_mode == 0:
            self.camera_control.stop()
        elif self.current_mode == 1:
            self.flex_control.stop()
        elif self.current_mode == 2:
            self.mpu_control.stop()

    def open_custom_gestures_dialog(self):
        dialog = CustomGesturesDialog(self.flex_control.activation_values,
                                      self.flex_control.current_functions, self)
        dialog.functions_updated.connect(self.update_gesture_mappings)
        dialog.exec_()

    def update_gesture_mappings(self, func_list):
        self.flex_control.current_functions = func_list
        for i, label in enumerate(self.gesture_labels):
            label.setText(f"Gesture {i + 1}: {func_list[i]}")

    def calibrate_mpu(self):
        # This would need to be implemented with actual calibration logic
        QMessageBox.information(self, "Calibration", "MPU6050 calibration would start here!")

    def closeEvent(self, event):
        self.stop_current_mode()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
