"""
By Sherrie Shen, 2017
Last modified Dec 13, 2017

Training knn using samples and responses from font_training_data.py
A classifier clf is then created for knn prediction
Related: font_training_data.py, predict_knn.py
"""

import cv2

from font_training_data import samples, responses

print "Performing the training"
clf = cv2.ml.KNearest_create()

clf.train(samples, cv2.ml.ROW_SAMPLE, responses)

print "KNN training complete"
