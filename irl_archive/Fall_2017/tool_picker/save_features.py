#!/usr/bin/env python

import cv2
from sklearn.cluster import KMeans
from sklearn.svm import SVC
import numpy as np
import pickle

# tool_list is a list of tuples containing tool type and image numbers
# tool_list = [('caliper',31), ('wirecutter', 36),('screwdriver',74),('hammer',37)]
tool_list = [('caliper',31),('screwdriver',31),('wirecutter',31),('hammer',31)]

# Create image paths and lists
image_paths = [] # initialize the list for image paths
image_classes = [] # initialize the list of image classes
class_id = 0
for tool, num_tool in tool_list:
    for num in range(1,num_tool):
        full_address = 'pngdataset/' + tool + '/' + str(num) + '.png'
        image_paths.append(full_address)
        image_classes.append(tool)
        class_id += 1

# Create feature extraction and keypoint detector objects
sift = cv2.xfeatures2d.SIFT_create(contrastThreshold=0.05)

# List where all the descriptors are stored
des_list = []
# Reading the image and calculating the features and corresponding descriptors
for image_path in image_paths:
    im = cv2.imread(image_path)
    gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    kpts, des = sift.detectAndCompute(gray, None) # Computing the key points and the descriptors
    des_list.append((image_path, des))  # Appending all the descriptors into the single list

# Stack all the descriptors vertically in a numpy array
descriptors = des_list[0][1]
for image_path, descriptor in des_list[1:]:
    descriptors = np.vstack((descriptors, descriptor))  # Stacking the descriptors

print('Descriptors Created')

# Perform k-means clustering
k = 300
kmeans = KMeans(n_clusters=k, random_state=0).fit(descriptors)
print('K-means clustering completed')

im_features = np.zeros((len(image_paths),k),'float32')
for i in range(len(image_paths)):
    for point in des_list[i][1]:
        features_cluster = kmeans.predict([point])
        im_features[i][features_cluster] +=1

print('Feature histogram created')

for i in range(len(im_features)):
    norm = np.linalg.norm(im_features[i])
    im_features[i] = im_features[i] / norm

print('Feature normalization completed')

svc = SVC().fit(im_features, image_classes)
print('SVC completed')

pickle.dump(kmeans, open('kmeans_model','wb'))
pickle.dump(svc, open('svc_model','wb'))
print('Finish saving models')
