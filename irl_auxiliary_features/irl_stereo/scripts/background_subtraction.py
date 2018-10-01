import cv2
import numpy as np
from bgknn.build.libopencv_bgknn import BackgroundSubtractorKNN

class BackgroundSubtractor:
    def __init__(self):
        #self.fgbg = cv2.BackgroundSubtractorMOG2()
        self.fgbg = BackgroundSubtractorKNN(400,500,True)
        self.shape = None
        self.mask = None
    def apply(self,image):
        if image.shape is not self.shape:
            self.mask = np.zeros(image.shape)
        return self.fgbg.apply(image, 0.1)
