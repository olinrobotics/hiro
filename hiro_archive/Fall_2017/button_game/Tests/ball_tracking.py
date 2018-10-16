# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
# defines max. size of tail that holds ball (x, y) coordinates
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)

blueLower = (110, 50, 50)
blueUpper = (130, 255, 255)

yellowLower = (23, 41, 133)
yellowUpper = (40, 150, 255)

redLower = (160, 140, 50)
redUpper = (179, 255, 255)

pts = deque(maxlen=args["buffer"])


# grab the reference to the webcam
camera = cv2.VideoCapture(0)

#keep looping
while True:
    #grab the current frame
    (grabbed, frame) = camera.read()

    # resize the frame, blur it, and return
    # it in the HSV color space
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # create a mask for the color green and then
    # perform erodions and dialations to make the
    # tracking more smooth

    mask_blue = cv2.inRange(hsv, blueLower, blueUpper)
    mask_green = cv2.inRange(hsv, greenLower, greenUpper)
    mask_yellow = cv2.inRange(hsv, yellowLower, yellowUpper)
    mask_red = cv2.inRange(hsv, redLower, redUpper)

    mask_total = mask_blue + mask_green + mask_yellow + mask_red

    mask_total = cv2.erode(mask_total, None, iterations=2)
    mask_total = cv2.dilate(mask_total, None, iterations=2)

    # find contours in the mask and initialize the (x, y) position
    # of the center of the ball
    cnts = cv2.findContours(mask_total.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    # only proceed if at least one contour found
    if len(cnts) > 0:
        # find largest contour area, compute minimum enclosing circle
        # and find its center
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if radius meets minimum size
        if radius > 30:
            print(radius)
            # draw circle outline and cente
            cv2.circle(frame, (int(x), int(y)), int(radius), (0,255,255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

    # update points in queue
    pts.appendleft(center)

    # loop over set of tracked points
    for i in xrange(1, len(pts)):
        # if either adjacent tracked points are none ignore them
        if pts[i - 1] is None or pts[i] is None:
            continue

        # compute line's thickness and draw it
        thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, pts[i -1], pts[i], (0, 0, 225), thickness)

    # show frame to screen
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask_total)
    key = cv2.waitKey(1) & 0xFF

    # q key will stop loop
    if key == ord("q"):
        break

# turn off camera and close any open windows
camera.release()
cv2.destoryAllWindows
