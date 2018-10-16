#!/usr/bin/python
import cv2
import time
import Image
import roslib; roslib.load_manifest('edwin')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from cv2 import cv

class ImageConverter:
    def __init__(self):
        self.inputMode = False
        self.roiPts = []
        self.roiBox = None
        self.knocked_over = False

        self.turn = "user"

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.track_object)
        self.pub = rospy.Publisher("whos_turn", String, queue_size=1)

        self.fgbg = cv2.BackgroundSubtractorMOG()

        # setup the mouse callback
        cv2.namedWindow("frame")
        cv2.setMouseCallback("frame", self.select_ROI)
        self.termination = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)

    def select_ROI(self, event, x, y, flags, param):
        if self.inputMode and event == cv2.EVENT_LBUTTONDOWN and len(self.roiPts) < 4:
            self.roiPts.append((x, y))
            cv2.circle(self.frame, (x, y), 4, (0, 255, 0), 2)
            cv2.imshow("frame", self.frame)

    def track_object(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        # initialize the termination criteria for cam shift, indicating
        # a maximum of ten iterations or movement by a least one pixel
        # along with the bounding box of the ROI
        # keep looping over the frames

        # if the see if the ROI has been computed
        if self.roiBox is not None:
            # convert the current frame to the HSV color space
            # and perform mean shift
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            backProj = cv2.calcBackProject([hsv], [0], self.roiHist, [0, 180], 1)

            # apply cam shift to the back projection, convert the
            # points to a bounding box, and then draw them
            (r, self.roiBox) = cv2.CamShift(backProj, self.roiBox, self.termination)
            pts = np.int0(cv2.cv.BoxPoints(r))
            num = pts[0][0] - pts[3][0]
            # print num
            if abs(num) > 70:
                self.turn = "edwin"
                # print "edwin's turn"
            else:
                self.turn = "user"
                # print "sophie's turn"
            cv2.polylines(self.frame, [pts], True, (0, 255, 0), 2)

        # show the frame and record if the user presses a key
        cv2.imshow("frame", self.frame)
        key = cv2.waitKey(1) & 0xFF

        # handle if the 'i' key is pressed, then go into ROI
        # selection mode
        if key == ord("i") and len(self.roiPts) < 4:
            # indicate that we are in input mode and clone the
            # frame
            self.inputMode = True
            orig = self.frame.copy()

            # keep looping until 4 reference ROI points have
            # been selected; press any key to exit ROI selction
            # mode once 4 points have been selected
            while len(self.roiPts) < 4:
                cv2.imshow("frame", self.frame)
                cv2.waitKey(0)

            # determine the top-left and bottom-right points
            self.roiPts = np.array(self.roiPts)
            s = self.roiPts.sum(axis = 1)
            tl = self.roiPts[np.argmin(s)]
            br = self.roiPts[np.argmax(s)]

            # grab the ROI for the bounding box and convert it
            # to the HSV color space
            roi = orig[tl[1]:br[1], tl[0]:br[0]]
            roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            #roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)

            # compute a HSV histogram for the ROI and store the
            # bounding box
            self.roiHist = cv2.calcHist([roi], [0], None, [16], [0, 180])
            self.roiHist = cv2.normalize(self.roiHist, self.roiHist, 0, 255, cv2.NORM_MINMAX)
            self.roiBox = (tl[0], tl[1], br[0], br[1])


def main():
    ic = ImageConverter()
    rospy.init_node('imageconverter')
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        ic.pub.publish(ic.turn)
        r.sleep()

if __name__ == "__main__":
    main()
