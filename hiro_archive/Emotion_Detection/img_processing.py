''' Image Processing Functions
    Author: Matthew Brucker
    Email: matthew.brucker@students.olin.edu
    Maintainer: Connor Novak
    Email: connor@students.olin.edu
    Purpose: Provide processing functions using OpenCV for handwriting_recognition.py
    '''

import cv2
import numpy as np
from Character import Character

def get_text_roi(frame, show_window=True):
    '''
        ----------------------------------------
        DESC: draws box around text in image
        ----------------------------------------
        ARGS:
        frame - image - image to search and find text
        show_window=True - boolean - represents whether or not to show debug
        image for chars (optional input)
        ----------------------------------------
        RETURNS: chars - list of characters found in image text
        ----------------------------------------
        '''

    kernel_sharpen = np.array([[-1,-1,-1],
                               [-1, 9,-1],
                               [-1,-1,-1]])

    kernel_sharpen_3 = np.array([[-1,-1,-1,-1,-1],
                                 [-1, 2, 2, 2,-1],
                                 [-1, 2, 8, 2,-1],
                                 [-1, 2, 2, 2,-1],
                                 [-1,-1,-1,-1,-1]]) / 8.0

    chars = [] # Stores found characters
    bound = 5
    kernel = np.ones((2,2),np.uint8)

    frame_blur = cv2.GaussianBlur(frame,(5,5),0)
    frame_gray = cv2.cvtColor(frame_blur,cv2.COLOR_BGR2GRAY) # Filter to grayscale
    frame_gray = cv2.filter2D(frame_gray.copy(),-1,kernel_sharpen_3) # Filter to sharpen
    # frame_gray = cv2.GaussianBlur(frame_gray, (5,5),0) # Gaussian blur to remove noise

    # Adaptive threshold to find numbers on paper
    thresh = cv2.adaptiveThreshold(frame_gray,255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,255,7)
    thresh = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel,iterations=2)
    #cv2.imshow('thresh',thresh)

    contours,hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(frame,contours,-1,(255,0,0),2)

    # Build the list of symbol contours and locations
    if len(contours) < 35: # If reasonable # of contours

        # For each contour, draw bounding rectangle, make sure it's a reasonable
        # size, make it a new frame,
        for ind,contour in enumerate(contours):
            [x,y,w,h] = cv2.boundingRect(contour)
            x_bound = w * .1;
            y_bound = h * .1;
            if  bound < x < (frame_gray.shape[1] - bound) and bound < y < (frame_gray.shape[0] - bound) and (x+w) <= (frame_gray.shape[1] - bound) and (y+h) <= (frame_gray.shape[0] - bound):
                roi = frame_gray[y-bound:y+h+bound,x-bound:x+w+bound]

                if len(roi) > 0: # Gets rid of weird zero-size contours
                    new_roi = cv2.adaptiveThreshold(roi,255,
                        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,35,7)
                    new_roi = cv2.morphologyEx(new_roi,cv2.MORPH_OPEN,kernel,iterations=2) # Embiggen numbers
                    new_roi = cv2.resize(new_roi, (20,20), interpolation=cv2.INTER_AREA) # standardize contours
                    deskewed = deskew(new_roi)
                    # Record contour and contour location, and filter out internal contours
                    if hierarchy[0][ind][3] == -1:
                        cont = Character(deskewed,(x,y,w,h), hog(deskewed))
                        chars.append(cont)

    if show_window:
        # Build an image to show all number contours
        num_len = len(chars)
        if num_len < 35 and num_len > 0:
            new_img = np.ones((20,20*num_len),np.uint8) # Creates matrix of ones
            y = 0 # counter
            for x in chars:
                new_img[:,y:y+20] = x.img
                y += 20
            cv2.imshow('image3',new_img)
    return chars

    # Deskews a 20x20 character image

def deskew(img):
    SZ = 20
    affine_flags = cv2.WARP_INVERSE_MAP|cv2.INTER_LINEAR
    m = cv2.moments(img)
    if abs(m['mu02']) < 1e-2:
        return img.copy()
    # print m
    skew = m['mu11']/m['mu02']
    M = np.float32([[1,skew,-0.5*SZ*skew], [0,1,0]])
    img = cv2.warpAffine(img,M,(SZ,SZ),flags=affine_flags)
    return img

    # Retursn the HOG for a given imagej

def hog(img):
    """ DOCSTRING:
        given img, return Histogram-Oriented Gradient (HOG) of img
        source: http://www.learnopencv.com/histogram-of-oriented-gradients/
        """
    bin_n = 16 # Number of bins in histogram
    gx = cv2.Sobel(img, cv2.CV_32F,1,0) # x gradient
    gy = cv2.Sobel(img, cv2.CV_32F,0,1) # y gradient
    mag,ang = cv2.cartToPolar(gx,gy) # Polar gradients
    bins = np.int32(bin_n*ang/(2*np.pi)) # creating binvalues
    bin_cells = bins[:10,:10],bins[10:,:10],bins[:10,10:],bins[10:,10:]
    mag_cells = mag[:10,:10],mag[10:,:10],mag[:10,10:],mag[10:,10:]
    hists = [np.bincount(b.ravel(), m.ravel(), bin_n) for b, m in zip(bin_cells, mag_cells)]
    hist = np.hstack(hists) # hist is a 64-bit vector
    return hist

def get_paper_region(img):
    screenCnt = None
    x_val = img.shape[1]
    y_val = img.shape[0]
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5,5),0) # Blur to remove noise
    edges = cv2.Canny(gray,75,220) # Line detect for edges of paper
    (cnts, _) = cv2.findContours(edges.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:5] # Sort contours by size
    for c in cnts:
        peri = cv2.arcLength(c,True)
        approx = cv2.approxPolyDP(c, 0.02*peri,True) # approximate the contour
        if len(approx) == 4: # Finds largest rectangular contour
            screenCnt = approx
            break
    if screenCnt is not None: # Make sure there is a paper contour
        pts = screenCnt.reshape(4,2) # Get the points of the contour
        rect = np.float32(np.zeros((4,2))) # Create new array of points in sorted order

        # Computes the top-left and bottom-right points using the sum of x/y coordinates
        s = pts.sum(axis = 1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]

        # Computes the top-right and bottom-left points based on the difference between x and y values
        diff = np.diff(pts,axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]

        # Extract the points of the rectangle and compute new image height/width
        (tl,tr,br,bl) = rect
        widthA = np.sqrt(((br[0]-bl[0]) ** 2) + ((br[1]-bl[1])**2))*2
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))*2
        # Compute potential height based on max/min y values of rect points
        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))*2
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))*2

        maxWidth = max(int(widthA),int(widthB))
        maxHeight = max(int(heightA),int(heightB))

        # Build our output image
        dst = np.float32(np.array([
            [0,0],
            [maxWidth-1,0],
            [maxWidth-1,maxHeight-1],
            [0, maxHeight-1]
        ]))
        perspective = cv2.getPerspectiveTransform (rect,dst)
        warp = cv2.warpPerspective(img.copy(),perspective,(maxWidth,maxHeight))
        cv2.imshow('warped',warp)
        return warp
    return img

def get_edwin_vision(img):
    """ DOCSTRING:
        Given img, returns img - section obscured by Edwin's eyelid
        """
    height, width, channels = img.shape
    crop_img = img[0:400, 0:width] # [y:height, x:width]
    cv2.imshow
    cv2.imshow('Cropped_Img', crop_img)
    return crop_img
