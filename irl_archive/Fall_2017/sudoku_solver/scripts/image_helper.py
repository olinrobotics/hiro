"""
by Khang Vu & Sherrie Shen, 2017
Last modified Dec 13, 2017

Helper file to handle images
"""
import numpy as np

import cv2
from skimage.feature import hog
from sklearn import preprocessing


def extract_hog_28(samples):
    list_hog_fd = []
    for i, sample in enumerate(samples):
        if i % 5000 == 0:
            print "Working on sample #%i" % i
        fd = hog(sample.reshape((28, 28)), orientations=9, pixels_per_cell=(14, 14), cells_per_block=(1, 1),
                 visualise=False, block_norm='L2-Hys')
        list_hog_fd.append(fd)

    hog_features = np.array(list_hog_fd, 'float32')

    # Normalize the features
    pp = preprocessing.StandardScaler().fit(hog_features)
    hog_features = pp.transform(hog_features)

    return hog_features, pp


def show_image(img):
    cv2.imshow('Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def show_images(imgs):
    for i, img in enumerate(imgs):
        cv2.imshow('Image %i' % i, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def put_to_black(img, x, y, w, h):
    roi = img[y:y + h, x:x + w]
    if h > w:
        b_size = int(h * 1.4)
    else:
        b_size = int(w * 1.4)

    b_y = int((b_size - h) / 2)
    b_x = int((b_size - w) / 2)
    black_image = np.zeros((b_size, b_size), np.uint8)
    black_image[b_y: b_y + h, b_x: b_x + w] = roi

    return black_image
