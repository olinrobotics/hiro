import numpy as np
import cv2


def main(im_in):
    img = cv2.imread(im_in)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

    # noise removal
    kernel = np.ones((3,3),np.uint8)
    opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)

    # sure background area
    sure_bg = cv2.dilate(opening,kernel,iterations=3)
    contours, hierarchy = cv2.findContours(sure_bg,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)


    # Find the index of the largest contour
    areas = [cv2.contourArea(c) for c in contours]
    if max(areas) > 10000:
        print im_in, ": FOUND HAND"
    else:
        print im_in, ": NO HAND"


if __name__ == "__main__":
    fn = "1, 2, 3, 4, 5, 6, 1a, 2a, 3a, 4a, 5a, 6a, 7a, 8a, 9a"
    fn_list = fn.split(", ")
    for elem in fn_list:
        main("test_imgs/"+elem+".jpg")