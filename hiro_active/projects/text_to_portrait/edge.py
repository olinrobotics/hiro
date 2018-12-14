import numpy as np
import cv2 as cv
import shutil
from matplotlib import pyplot as plt
from images import fetch_image

print('Enter Name:')
name = raw_input()

# Remove previous images and their directory
shutil.rmtree('images/')

# Download images from google
fetch_image(name)

for i in range(1):
# for i in range(10):
    # READING IMAGE
    img = cv.imread('images/' + name + '/' + name + '_' + str(i) + '.jpg', 0)

    # Skip over any corrupted images
    if img is None:
        continue

    height, width = img.shape[:2]

    # CANNY EDGE DETECTION
    # edges	= cv.Canny( image, threshold1, threshold2[, edges[, apertureSize[, L2gradient]]] )
    edges = cv.Canny(img, 100, 200)
    
    # GET XY COORDINATES
    indices = np.where(edges != [0])
    coordinates = zip(indices[1], indices[0])

    # print(coordinates)

    print(width, height)

    # Create a black image of original image height and width
    draw_img = np.zeros((height, width, 3), np.uint8)

    # DRAW POINTS
    for x, y in coordinates:
        cv.line(draw_img, (x, y), (x+1, y+1), (255,255,255), 2)

    plt.imshow(draw_img, cmap = 'gray', interpolation = 'bicubic')
    plt.show()