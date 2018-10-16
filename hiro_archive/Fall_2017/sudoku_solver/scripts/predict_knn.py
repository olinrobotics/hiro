"""
By Sherrie Shen & Khang Vu, 2017
Last modified Dec 13, 2017

Using knn to predict digits in an image
This will use the classifier from training_knn.py
Related: training_knn.py
"""
import numpy as np

import cv2

import image_helper as imhelp
from training_knn import clf


def recognize_one(im=None, image_path=None, will_show_img=False, adapt_threshold=True):
    """
    Recognizes a digit in an image
    :param (array) im: input image. If None, use image_path
    :param (String) image_path: link to the image to recognize. Only used when im is None
    :param (bool) will_show_img: whether to show the image at the end of the func
    :param (bool) adapt_threshold: whether to process adaptive threshold
    :return: (int) a predicted digit
    """
    # Read the input image
    if im is None:
        im = cv2.imread(image_path)
        if im is None:
            return None

    # Returned result
    result = 0

    # If the image is too big, resize it
    max_size = 800.0
    if im.shape[0] > max_size or im.shape[1] > max_size:
        if im.shape[0] > im.shape[1]:
            ratio = max_size / im.shape[0]
        else:
            ratio = max_size / im.shape[1]
        im = cv2.resize(im, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_AREA)

    # Output image
    out = np.zeros(im.shape, np.uint8)

    if adapt_threshold:
        # Image process: blur, binary, threshold
        blur = cv2.GaussianBlur(im, (11, 11), 0)
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
    else:
        th = im.copy()

    # Find contours in the image
    im2, contours, hierarchy = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Finding the max contours' area, which contains the digit
    max_area = 0
    max_cnt = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 50 and max_area < area:
            max_area = area
            max_cnt = cnt

    # No appropriate contours found means there is no digit, return 0
    if max_cnt is None:
        return result

    # max_cnt should contain the digit
    [x, y, w, h] = cv2.boundingRect(max_cnt)

    # Put the image into a black image
    black_image = imhelp.put_to_black(th, x, y, w, h)

    # Resize the image
    roi = cv2.resize(black_image, (28, 28), interpolation=cv2.INTER_AREA)
    roi = cv2.dilate(roi, (10, 10), iterations=1)

    # Reshape the image
    roi.shape = (1, 784)
    roi = np.float32(roi)

    # Predict the image
    retval, results, neigh_resp, dists = clf.findNearest(roi, k=10)

    # Put text on the output image
    result = int((results.ravel()))
    cv2.putText(out, str(result), (x, y + h), 0, 1, (255, 255, 255))

    if will_show_img:
        # Show the output image, just for testing
        imhelp.show_images([im, out])

    return result


def recognize_all(im=None, image_path=None, will_show_img=False):
    """
    Recognizes all digits in an image
    :param (array) im: input image. If None, use image_path
    :param (String) image_path: link to the image to recognize. Only used when im is None
    :param (bool) will_show_img: whether to show the image at the end of the func
    :return: None
    """
    # Read the input image
    if im is None:
        im = cv2.imread(image_path)
        if im is None:
            return None

    # If the image is too big, resize it
    max_size = 800.0
    if im.shape[0] > max_size or im.shape[1] > max_size:
        if im.shape[0] > im.shape[1]:
            ratio = max_size / im.shape[0]
        else:
            ratio = max_size / im.shape[1]
        im = cv2.resize(im, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_AREA)

    # Output image
    out = np.zeros(im.shape, np.uint8)

    # Image process: blur, binary, threshold
    blur = cv2.GaussianBlur(im, (11, 11), 0)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

    # Find contours in the image
    im2, contours, hierarchy = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        if cv2.contourArea(cnt) > 50:
            [x, y, w, h] = cv2.boundingRect(cnt)
            if h > 20 and w > 5:
                # Draw the rectangles in the original image
                cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Put the image into a black image
                black_image = imhelp.put_to_black(th, x, y, w, h)

                # Resize the image
                roi = cv2.resize(black_image, (28, 28), interpolation=cv2.INTER_AREA)
                roi = cv2.dilate(roi, (10, 10), iterations=1)

                # imhelp.show_image(roi)

                # Reshape the image
                roi.shape = (1, 784)
                roi = np.float32(roi)

                # Predict the image
                retval, results, neigh_resp, dists = clf.findNearest(roi, k=10)

                # Put text on the output image
                string = str(int((results.ravel())))
                cv2.putText(out, string, (x, y + h), 0, 1, (255, 255, 255))

    if will_show_img:
        # Show the output image, just for testing
        imhelp.show_images([im, out])


if __name__ == "__main__":
    recognize_one(image_path="test_imgs/photo_4.jpg", will_show_img=True)
    # recognize_all(image_path="test_imgs/photo_3.jpg", will_show_img=True)
