import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while(1):

    _, frame = cap.read()


    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])

    lower_green = np.array([40,40,40])
    upper_green = np.array([80,255,255])

    mask_b = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_g = cv2.inRange(hsv, lower_green, upper_green)
    mask_t = mask_b + mask_g

    res = cv2.bitwise_and(frame,frame, mask= mask_t)


    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask_t)
    cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
