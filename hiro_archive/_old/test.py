import cv2
import numpy as np

im = cv2.imread('photo2.JPG')
gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)

imgSplit = cv2.split(im)
flag,b = cv2.threshold(imgSplit[2],0,255,cv2.THRESH_OTSU)

element = cv2.getStructuringElement(cv2.MORPH_CROSS,(1,1))
cv2.erode(b,element)

edges = cv2.Canny(b,150,200,3,5)

while(True):

    img = im.copy()

    lines = cv2.HoughLinesP(edges,1,np.pi/2,2, minLineLength = 620, maxLineGap = 100)[0]

    for x1,y1,x2,y2 in lines:
        cv2.line(img,(x1,y1),(x2,y2),(0,255,0),1)

    cv2.imshow('houghlines',img)

    if k == 27:
        break

cv2.destroyAllWindows()

filename = 'chessboard.png'
    5 img = cv2.imread(filename)
    6 gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    7
    8 gray = np.float32(gray)
    9 dst = cv2.cornerHarris(gray,2,3,0.04)
   10
   11 #result is dilated for marking the corners, not important
   12 dst = cv2.dilate(dst,None)
   13
   14 # Threshold for an optimal value, it may vary depending on the image.
   15 img[dst>0.5*dst.max()]=[0,0,255]
   16
   17 cv2.imshow('dst',img)
   18 if cv2.waitKey(0) & 0xff == 27:
   19     cv2.destroyAllWindows()







