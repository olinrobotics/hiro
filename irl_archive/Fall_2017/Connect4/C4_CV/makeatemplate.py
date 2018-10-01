import cv2
import numpy as np
from matplotlib import pyplot as plt

imgray = cv2.imread('templatepic.png', 0)
img = cv2.medianBlur(imgray,5)

th1 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)

while True:
    cv2.imshow('frame', th1)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        destroyAllWindows
    elif k == ord('s'):
        cv2.imwrite('template.png', th1)
