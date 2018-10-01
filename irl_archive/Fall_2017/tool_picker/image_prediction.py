#!/usr/bin/env python

from sklearn.svm import SVC
import numpy as np
import pickle
import cv2

def predict(image_path):
    k = 300 # number of k means clusters

    # load pretrained model from file
    kmeans = pickle.load(open('kmeans_model','rb'))
    svc = pickle.load(open('svc_model','rb'))

    # create sift detection object
    sift = cv2.xfeatures2d.SIFT_create(contrastThreshold=0.05)

    # read image and calculate descriptor
    im = cv2.imread(image_path)
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    kpts, des = sift.detectAndCompute(gray, None)

    # create feature histogram
    im_features = np.zeros(k,'float32')
    for point in des:
        features_cluster = kmeans.predict([point])
        im_features[features_cluster] += 1

    # normalize feature vector
    norm = np.linalg.norm(im_features)
    im_features = im_features / norm

    return svc.predict([im_features])

if __name__ == "__main__":
    for i in range(1,31):
        image_path = 'pngdataset/caliper/'+ str(i) + '.png'
        result = predict(image_path)
        print(result)
