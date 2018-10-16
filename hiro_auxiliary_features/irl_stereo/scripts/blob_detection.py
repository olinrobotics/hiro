import cv2
import numpy as np

def circleArea(r):
    return r*r*3.14159265358979

class BlobDetector:
    def __init__(self):
        #BLOB DETECTION ...
        params = cv2.SimpleBlobDetector_Params()
        params.minDistBetweenBlobs = 0

        params.filterByColor = False 

        params.filterByArea = True 
        params.minArea = 3000.
        params.maxArea = 999999999.

        params.filterByCircularity = False

        params.filterByConvexity = True 
        params.minConvexity = 0.8

        params.filterByInertia = False

        self.detector = cv2.SimpleBlobDetector(params)

    def apply(self, img):
        #identified = np.zeros(image.shape, dtype=image.dtype)
        k_dilate = np.asarray([
            [.07,.12,.07],
            [.12,.24,.12],
            [.07,.12,.07]
            ],np.float32)

        proc = cv2.dilate(img, k_dilate, iterations = 5) # fill the holes

        identified = proc.copy()

        labels = self.detector.detect(proc)

        if len(proc.shape) == 3 and proc.shape[2] == 3:
            # BGR --> GRAY
            proc = cv2.cvtColor(proc, cv2.COLOR_BGR2GRAY)

        ctrs, hrch = cv2.findContours(proc, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        id_ctrs = []

        for ctr in ctrs:
            a = cv2.contourArea(ctr)
            if 3000 < a and a < 1000000:
                id_ctrs.append(ctr)

        # Fill in Contour with average value
        l = []
        for i in range(len(id_ctrs)):
            cimg = np.zeros_like(proc)
            cv2.drawContours(cimg, id_ctrs, i, color=255, thickness=-1)
            pts = np.where(cimg == 255)
            t = np.average(img[pts[0], pts[1]])
            l.append(t)
            
        if len(l) > 0:
            i = np.argmax(l)
            ctr = id_ctrs[i]
            x,y,w,h = cv2.boundingRect(ctr)
            m = cv2.moments(ctr)
            a = cv2.contourArea(ctr)
            return x,y,w,h,m,a
        return None

        #cv2.drawContours(identified, id_ctrs ,-1,(255,0,0),3)

        #identified = cv2.drawKeypoints(identified,labels,flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        #return labels, identified

