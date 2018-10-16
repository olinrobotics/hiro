import numpy as np
import cv2
from matplotlib import pyplot as plt

"""These functions all involve image manipulation with open CV"""

def showImage():
    img = cv2.imread('/home/mark/Desktop/RMills.jpg',1)
    cv2.imshow('image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def saveImage():
    img = cv2.imread('/home/mark/Desktop/Rect.gif',1)
    cv2.imwrite('/home/mark/Desktop/Rect.jpg',img)

def exitOrSaveAndExit():
    img = cv2.imread('/home/mark/Desktop/RMills.jpg',0)
    cv2.imshow('image',img)
    k = cv2.waitKey(0)
    if k == 27:
        cv2.destroyAllWindows()
    elif k == ord('s'):
        cv2.imwrite('/home/mark/Desktop/RMills.png',img)
        cv2.destroyAllWindows

def displayWithMatplotlib():
    img = cv2.imread('/home/mark/Desktop/RMills.jpg',0)
    plt.imshow(img,cmap = 'gray', interpolation = 'bicubic')
    plt.xticks([]), plt.yticks([])
    plt.show()

"""Functions that have to do with video"""

def displayVideo():
    cap = cv2.VideoCapture(0)

    while(True):
        #Capture frame-by-frame
        ret, frame = cap.read()

        #Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #Display the resulting frame
        cv2.imshow('frame',gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    #When everything done, release the Capture
    cap.release()
    cv2.destroyAllWindows()

saveImage()
