import numpy as np
import cv2
import sys
class ColorFinder(object):
    def __init__(self):
        self.win = cv2.namedWindow('img', 1)
        self.col_win = cv2.namedWindow('col')
        cv2.setMouseCallback('img', self.mouse_cb)
        self.img = None
        self.col = np.zeros((128,128,3),dtype=np.uint8)

    def mouse_cb(self,ev,x,y,flag,data):
        cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV, dst=self.hsv)
        self.col[:,:,:] = self.img[y,x]

        if flag == 1: # left mouse down
            print '-----------------------------------'
            print 'rgb :', self.img[y,x]
            print 'hsv :', self.hsv[y,x]

    def help(self):
        print '==============================================='
        print 'Drag your mouse around the window.'
        print 'An Alternative Window will display the color of the region that the mouse is over.'
        print 'When you click, a corresponding hsv value will be printed.'
        print '==============================================='

    def run(self, dev=0):
        self.help()
        cam = cv2.VideoCapture(dev)
        succ, img = cam.read()

        if succ:
            self.img = img
            self.hsv = np.zeros_like(img)

        while True:
            succ, img = cam.read()
            if succ:
                self.img = img
                cv2.imshow('col', self.col)
                cv2.imshow('img', self.img)
            if cv2.waitKey(10) == 27:
                return

if __name__ == "__main__":
    dev = 0
    if len(sys.argv) > 1:
        dev_str = sys.argv[1]
        try:
            if dev_str.startswith('/dev/video'):
                dev = int(dev_str.split('/dev/video')[1])
            else:
                dev = int(dev_str)
        except Exception:
            dev = 0
    ColorFinder().run(dev)
