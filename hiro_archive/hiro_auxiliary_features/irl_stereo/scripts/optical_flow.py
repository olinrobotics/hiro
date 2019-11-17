import cv2
import numpy as np

class OpticalFlow:
    def __init__(self,img):
        # expects BGR Image 
        self.prv = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        self.hsv = np.zeros_like(img)
        self.hsv[...,1] = 255

    def apply(self,img):
        # calculate optical flow; also expects BGR
        nxt = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        flow = cv2.calcOpticalFlowFarneback(self.prv,nxt, 0.5, 3, 15, 3, 5, 1.2, 0)
        mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
        self.hsv[...,0] = ang*180/np.pi/2
        self.hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
        bgr = cv2.cvtColor(self.hsv,cv2.COLOR_HSV2BGR)
        # update previous frame
        self.prv = nxt 
        return bgr
