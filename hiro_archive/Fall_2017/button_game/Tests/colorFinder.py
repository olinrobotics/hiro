import os
import argparse
import numpy as np
import cv2

def nothing(x):
    pass

# Create a black image, a window
img = np.zeros((300,512,3), np.uint8)
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('lh','image',0,255,nothing)
cv2.createTrackbar('hh','image',0,255,nothing)
cv2.createTrackbar('ls','image',0,255,nothing)
cv2.createTrackbar('hs','image',0,255,nothing)

cap = cv2.VideoCapture(0)


def main():
    while(True):
        # Capture frame-by-frame
        ret, img = cap.read()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #Initial Color Thresholding
        #--------------------------------------------------
        #Threshold for blue (93,173,173) (109,255,255)
        lower_blue = np.array([cv2.getTrackbarPos('lh','image'),cv2.getTrackbarPos('ls','image'),cv2.getTrackbarPos('ls','image')])
        upper_blue = np.array([cv2.getTrackbarPos('hh','image'),cv2.getTrackbarPos('hs','image'),cv2.getTrackbarPos('hs','image')])

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)


        # Bitwise-AND mask and original i\mage
        res = cv2.bitwise_and(img,img, mask= mask)

        #Threshold for yellow

        #Threshold for red

        #Threshold for green

        #--------------------------------------------------
        cv2.imshow('image', res)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
