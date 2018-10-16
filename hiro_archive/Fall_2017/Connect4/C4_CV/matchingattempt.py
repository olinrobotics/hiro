'''
Purpose: to process a board of Connect4
Authors: Hannah Kolano and Kevin Zhang
Contact: hannah.kolano@students.olin.edu
Last edited: 11/1/17
'''

import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt

'''initialize camera'''
cap = cv2.VideoCapture(0)
_, frame = cap.read()
imgray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) # grayscale image
_, imgray = cv2.threshold(imgray, 127,255,cv2.THRESH_BINARY)
'''define template'''
template = cv2.imread('templatepic.png', 0)
template = cv2.resize(template, (150,150))
w, h = template.shape[::-1]

'''blur image and find edges'''
#img = cv2.medianBlur(imgray,5)
#th1 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)

sift = cv2.xfeatures2d.SIFT_create()
kp1, des1 = sift.detectAndCompute(template,None)
kp2, des2 = sift.detectAndCompute(imgray,None)

bf = cv2.BFMatcher()
matches = bf.knnMatch(des1, des2, k=2)

good = []
for m, n in matches:
    if m.distance <0.75*n.distance:
        good.append([m])

print(good)
img3 = cv2.drawMatchesKnn(template,kp1,imgray,kp2,good, None,flags=2)
'''
res = cv2.matchTemplate(imgray, template, cv2.TM_CCOEFF_NORMED)
threshold = 0.5

loc = np.where(res >= threshold)
for pt in zip(*loc[::-1]):
    cv2.rectangle(frame, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)
'''
'''
__, contours, __ = cv2.findContours(th1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(th1, contours, -1, (0,255,0), 3)
'''
plt.imshow(img3), plt.show()

while(True):

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        destroyAllWindows()
    elif k == ord('s'):
        cv2.imwrite('corners.png', img)

#TODO:
'''find edges of board, normalize the view'''

#TODO:
'''Figure out what the current layout is like'''

#TODO:
'''for debugging, create visual output of board'''

#TODO:
'''Format to send to the machine learning part'''
