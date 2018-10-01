"""
By Khang Vu, 2017
Last modified Dec 13, 2017

This script takes a captured sudoku image and parses into a sudoku instance
It uses predict_knn to predict numbers in the sudoku image
Related: predict_knn.py, sudoku.py
"""
import predict_knn as knn
import numpy as np
import cv2
import image_helper as imhelp
from sudoku_algorithm.sudoku import Sudoku


def get_edges(cnt):
    """
    Helper function returns 4 edges from contours that mark the sudoku grid
    :param cnt: contours
    :return: an array of 4 points a, b, c, d
    :rtp: [(int, int)]
    """
    a = [-1, -1]
    b = [-1, -1]
    c = [-1, -1]
    d = [-1, -1]

    for element in cnt:
        point = element[0]
        if a[0] + a[1] > point[0] + point[1] or a[0] == -1:
            a = point

        if d[0] + d[1] < point[0] + point[1] or d[0] == -1:
            d = point

        if b[0] - b[1] < point[0] - point[1] or b[0] == -1:
            b = point

        if c[1] - c[0] < point[1] - point[0] or c[0] == -1:
            c = point

    return a, b, c, d


def get_grid(im=None, image_path=None):
    """
    Get a sudoku grid from a captured image
    :param im: image object
    :param (String) image_path: path to sudoku image
    :return: sudoku grid image after perspective transformation
    :rtp: [[uint8]]
    """
    if im is None:
        im = cv2.imread(image_path)

    if im is None:
        return

    # If the image is too big, resize it
    max_size = 800.0
    if im.shape[0] > max_size or im.shape[1] > max_size:
        if im.shape[0] > im.shape[1]:
            ratio = max_size / im.shape[0]
        else:
            ratio = max_size / im.shape[1]
        im = cv2.resize(im, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_AREA)

    # Image process: blur, binary, threshold
    blur = cv2.GaussianBlur(im, (11, 11), 0)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

    # Find contours in the image
    im2, contours, hierarchy = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    max_area = 0
    max_cnt = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 50 and max_area < area:
            max_area = area
            max_cnt = cnt

    a, b, c, d = get_edges(max_cnt)
    size = int(np.sqrt(max_area))

    pts1 = np.float32([a, b, c, d])
    pts2 = np.float32([[0, 0], [size, 0], [0, size], [size, size]])

    M = cv2.getPerspectiveTransform(pts1, pts2)
    sudoku_grid = cv2.warpPerspective(th, M, (size, size))

    imhelp.show_image(sudoku_grid)
    return sudoku_grid


def parse_sudoku(grid, n=4):
    """
    Parse a sudoku grid (image) into a Sudoku instance
    :param grid: an image
    :param n: whether the sudoku is 4x4 or 9x9
    :return: a Sudoku instance
    """
    box_len = len(grid) / n
    problem_set = []

    # Output image
    out = np.zeros(grid.shape, np.uint8)

    for i in range(n):
        for j in range(n):
            # Get cell(i, j) from the grid
            x = j * box_len
            y = i * box_len
            cell = grid[y: y + box_len, x: x + box_len]

            # Remove the border by extracting only 80% of the image
            x = int(box_len * 0.12)
            y = int(box_len * 0.12)
            w = h = int(box_len * 0.8)
            cell = cell[y: y + h, x: x + w]

            number = knn.recognize_one(im=cell, adapt_threshold=False)

            if 0 < number <= n:
                problem_set.append([i, j, number])
                cv2.putText(out, str(number), (j * box_len + h/2, i * box_len + h), 0, 1, (255, 255, 255))

    imhelp.show_image(out)
    return Sudoku(n=int(np.sqrt(n)), problem_set=problem_set)


def from_image(im=None, image_path=None, n=4):
    # Try to get a sudoku grid from an image
    sudoku_grid = get_grid(im=im, image_path=image_path)

    # Try to parse the grid into a sudoku object
    sudoku = parse_sudoku(grid=sudoku_grid, n=n)

    # Solve it
    sudoku.print_sudoku()
    sudoku.solve()
    sudoku.print_sudoku()

    return sudoku


if __name__ == "__main__":
    im = cv2.imread("test_imgs/sudoku_3.jpg")
    imhelp.show_image(im)
    sudoku = from_image(im=im, n=4)

    # Print solution
    print "r c number"
    for cell in sudoku.solution:
        print cell.row, cell.col, cell.get_number()
