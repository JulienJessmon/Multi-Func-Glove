"""import mouse
while True:
    print(mouse.get_position())"""

import cv2
from cvzone.HandTrackingModule import HandDetector
import mediapipe as mp
import time
import mouse
import pyautogui
import numpy as np
import threading

cap = cv2.VideoCapture(0)
cam_w, cam_h = 640, 480
cap.set(3, cam_w)
cap.set(4, cam_h)

detector = HandDetector(detectionCon=0.9, maxHands=1)

pTime = 0
cTime = 0

frameR = 30

l_delay =

while True:
    success, img = cap.read()
    img = cv2.flip(img, 1)
    hands, img = detector.findHands(img, flipType=False)
    cv2.rectangle(img, (frameR, frameR), (cam_w - frameR, cam_h - frameR), (255, 0, 255), 2)

    if hands:
        lmlist = hands[0]['lmList']
        ind_x, ind_y = lmlist[8][0], lmlist[8][1]
        mid_x, mid_y = lmlist[12][0], lmlist[12][1]
        cv2.circle(img, (ind_x, ind_y), 5, (255, 0, 0), 2)
        fingers = detector.fingersUp(hands[0])

        if fingers[1] == 1 and fingers[2] == 0 and fingers[0] == 1:
            conv_x = int(np.interp(ind_x, (frameR, cam_w - frameR), (0, 1366)))
            conv_y = int(np.interp(ind_y, (frameR, cam_h - frameR), (0, 768)))
            mouse.move(conv_x, conv_y)

        if fingers[1] == 1 and fingers[2] == 1 and fingers[0] == 1:
            if abs(ind_x-mid_x) < 25:


        cTime = time.time()
        fps = 1/(cTime - pTime)
        pTime = cTime

        cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_ITALIC, 2, (255, 255, 0), 3)

    cv2.imshow("image", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
