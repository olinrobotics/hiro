import cv2
import numpy as np

class Character:

    def __init__ (self, img, (x,y,w,h),HOG):
        self.result = -1
        self.img = img # B/W image data in a 20x20 numpy matrix
        self.x = x # x position
        self.y = y # y position
        self.w = w # Contour width
        self.h = h # Contour height
        self.HOG = HOG # Histogram of oriented gradients

        # Print function to help with debugging
    def __str__(self):
        return 'X: ' + str(self.x) + ' Y: ' + str(self.y) + ' W: ' +  \
        str(self.w) + ' H: ' + str(self.h)
