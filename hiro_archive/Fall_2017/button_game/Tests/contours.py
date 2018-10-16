import numpy as np
import cv2

im = cv2.imread('/home/mark/Desktop/Rect.jpg')
imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
ret,thresh = cv2.threshold(imgray,127,255,0)
image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

img = cv2.drawContours(im, contours, -1, (0,255,0), 3)


cv2.imshow('image',im)
if cv2.waitKey(0) & 0xFF == ord('q'):
    cv2.destroyAllWindows()
