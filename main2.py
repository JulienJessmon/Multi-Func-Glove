# this is made usin mp, left area shows no lagging in this

import cv2
# from cvzone.HandTrackingModule import HandDetector # mot usin since mediapipe only
import mediapipe as mp
import time
import mouse
import numpy as np
import threading

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.9, min_tracking_confidence=0.9)
mp_drawing = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)
cam_w, cam_h = 640, 480
cap.set(3, cam_w)
cap.set(4, cam_h)

pTime = 0
frameR = 100

l_delay = 0
r_delay = 0

l_last_click = 0
r_last_click = 0
click_delay = 1

current_time = time.time()

# Initialize smoothing variables
prev_conv_x, prev_conv_y = 0, 0
alpha = 0.7  # Smoothing factor (adjust between 0 to 1, closer to 1 is smoother)


def smooth_positions(prev_x, prev_y, new_x, new_y, alpha):
    # Low-pass filter: smooth the current position based on the previous one
    smoothed_x = alpha * prev_x + (1 - alpha) * new_x
    smoothed_y = alpha * prev_y + (1 - alpha) * new_y
    return smoothed_x, smoothed_y


def l_clk_delay():
    global l_delay
    global l_clk_thread
    time.sleep(1)
    l_delay = 0
    l_clk_thread = threading.Thread(target=l_clk_delay)


def r_clk_delay():
    global r_delay
    global r_clk_thread
    time.sleep(1)
    r_delay = 0
    r_clk_thread = threading.Thread(target=r_clk_delay)


l_clk_thread = threading.Thread(target=l_clk_delay)
r_clk_thread = threading.Thread(target=r_clk_delay)

while True:
    success, img = cap.read()
    if not success:
        print("Failed to grab frame")
        break

    img = cv2.flip(img, 1)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)

    cv2.rectangle(img, (frameR, frameR), (cam_w - frameR, cam_h - frameR), (255, 0, 255), 2)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            ind_x, ind_y = hand_landmarks.landmark[8].x * cam_w, hand_landmarks.landmark[8].y * cam_h
            mid_x, mid_y = hand_landmarks.landmark[12].x * cam_w, hand_landmarks.landmark[12].y * cam_h
            ring_x, ring_y = hand_landmarks.landmark[16].x * cam_w, hand_landmarks.landmark[16].y * cam_h

            current_time = time.time()

            cv2.circle(img, (int(ind_x), int(ind_y)), 5, (255, 0, 0), 2)

            fingers_up = [
                hand_landmarks.landmark[4].y < hand_landmarks.landmark[3].y,
                hand_landmarks.landmark[8].y < hand_landmarks.landmark[6].y,
                hand_landmarks.landmark[12].y < hand_landmarks.landmark[10].y,
                hand_landmarks.landmark[16].y < hand_landmarks.landmark[14].y,
                hand_landmarks.landmark[20].y < hand_landmarks.landmark[18].y
            ]

            if fingers_up[1] == 1 and fingers_up[2] == 0 and fingers_up[3] == 0 and fingers_up[4] == 0:
                conv_x = int(np.interp(ind_x, (frameR, cam_w - frameR), (0, 1536)))
                conv_y = int(np.interp(ind_y, (frameR, cam_h - frameR), (0, 864)))
                smoothed_x, smoothed_y = smooth_positions(prev_conv_x, prev_conv_y, conv_x, conv_y, alpha)
                prev_conv_x, prev_conv_y = smoothed_x, smoothed_y
                mouse.move(int(smoothed_x), int(smoothed_y))

            if fingers_up[1] == 1 and fingers_up[2] == 1 and fingers_up[3] == 0 and fingers_up[4] == 0:
                if abs(ind_x - mid_x) < 25:
                    if (current_time - l_last_click) > click_delay:
                        mouse.click(button='left')
                        l_last_click = current_time

            if fingers_up[1] == 1 and fingers_up[2] == 1 and fingers_up[3] == 1 and fingers_up[4] == 0:
                if abs(ind_x - mid_x) < 25 and abs(mid_x - ring_x) < 25:
                    if (current_time - r_last_click) > click_delay:
                        mouse.click(button='right')
                        r_last_click = current_time

    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime
    cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_ITALIC, 2, (255, 255, 0), 3)

    cv2.imshow("image", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
