import cv2
import numpy as np
from matplotlib import pyplot as plt

def Thresholding():
    img = cv2.imread('/home/mark/Desktop/gradient.png',0)
    ret, thresh1 = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
    ret,thresh2 = cv2.threshold(img,127,255,cv2.THRESH_BINARY_INV)
    ret,thresh3 = cv2.threshold(img,127,255,cv2.THRESH_TRUNC)
    ret,thresh4 = cv2.threshold(img,127,255,cv2.THRESH_TOZERO)
    ret,thresh5 = cv2.threshold(img,127,255,cv2.THRESH_TOZERO_INV)

    titles = ['Original Image','BINARY','BINARY_INV','TRUNC','TOZERO','TOZERO_INV']

    images = [img, thresh1, thresh2, thresh3, thresh4, thresh5]

    for i in xrange(6):
        plt.subplot(2,3,i+1),plt.imshow(images[i],'gray')
        plt.title(titles[i])
        plt.xticks([]),plt.yticks([])

    plt.show()

def Convolution():
    img = cv2.imread('/home/mark/Desktop/Apple.jpg')

    kernel = np.ones((5,5),np.float32) / 25
    dst = cv2.filter2D(img,-1,kernel)

    plt.subplot(121),plt.imshow(img),plt.title('Original')
    plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(dst),plt.title('Averaging')
    plt.xticks([]), plt.yticks([])
    plt.show()

def Averaging():
    img = cv2.imread('/home/mark/Desktop/Apple.jpg')

    blur = cv2.blur(img,(5,5))

    plt.subplot(121),plt.imshow(img),plt.title('Original')
    plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(blur),plt.title('Blurred')
    plt.xticks([]), plt.yticks([])
    plt.show()

def GaussianBlur():
    img = cv2.imread('/home/mark/Desktop/RMills.jpg')

    blur = cv2.GaussianBlur(img,(5,5),0)

    plt.subplot(121),plt.imshow(img),plt.title('Original')
    plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(blur),plt.title('Blurred')
    plt.xticks([]), plt.yticks([])
    plt.show()

def MedianFilter():
    img = cv2.imread('/home/mark/Desktop/Pic.jpg')

    median = cv2.medianBlur(img,5)

    plt.subplot(121),plt.imshow(img),plt.title('Original')
    plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(median),plt.title('Filtered')
    plt.xticks([]), plt.yticks([])
    plt.show()

def BilateralFilter():
    img = cv2.imread('/home/mark/Desktop/RMills.jpg')

    blur = cv2.bilateralFilter(img,9,75,75)

    plt.subplot(121),plt.imshow(img),plt.title('Original')
    plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(blur),plt.title('Filtered')
    plt.xticks([]), plt.yticks([])
    plt.show()



"""Morphological Transformations"""


def Erosion():
    img = cv2.imread('/home/mark/Desktop/j.png', 0)
    kernel = np.ones((5,5), np.uint8)
    erosion = cv2.erode(img, kernel, iterations=1)

    cv2.imshow('Original', img)
    cv2.imshow('Erosion', erosion)

    cv2.waitKey(0)

def Dilation():
    img = cv2.imread('/home/mark/Desktop/j.png', 0)
    kernel = np.ones((5,5), np.uint8)
    dilation = cv2.dilate(img, kernel, iterations=1)

    cv2.imshow('Original', img)
    cv2.imshow('Dilation', dilation)

    cv2.waitKey(0)

def MorphGradient():
    img = cv2.imread('/home/mark/Desktop/j.png', 0)
    kernel = np.ones((5,5), np.uint8)
    dilation = cv2.morphologyEx(img, cv2.MORPH_GRADIENT, kernel)

    cv2.imshow('Original', img)
    cv2.imshow('Dilation', dilation)

    cv2.waitKey(0)

GaussianBlur()
