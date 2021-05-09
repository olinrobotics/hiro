import numpy as np
import csv, cv2, os, statistics
from tqdm import tqdm


def canny_webcam():
    "Live capture frames from webcam and show the canny edge image of the captured frames."

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()  # ret gets a boolean value. True if reading is successful (I think). frame is an
        # uint8 numpy.ndarray

        frame = cv2.GaussianBlur(frame, (7, 7), 1.41)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        edge = cv2.Canny(frame, 25, 75)

        cv2.imshow('Canny Edge', edge)

        if cv2.waitKey(20) == ord('q'):  # Introduce 20 milisecond delay. press q to exit.
            break



def canny_draw_lines(frame):
    img = cv2.imread(frame)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(img_gray, 200, 250)

    contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    means = geometric_center(contours)

    for i in range(0,len(means)):
        cv2.circle(img, (round(means[i][0]), round(means[i][1])), 3, (0, 0, 0), -1)

    cv2.imshow('Edges', edges)
    cv2.waitKey(0)
    cv2.drawContours(img, contours, -1, (0, 255, 0), 3)

    cv2.imshow('contours', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return contours


def points_to_csv(points, num):
    file_name = f'grab_{num}.csv'
    with open(file_name, 'w', newline="") as f:
        writer = csv.writer(f)
        for i in range(0, len(points)):
            point = points[i]
            data = [point[0], point[1]]
            writer.writerow(data)


def geometric_center(contours):
    points = []
    means = []
    for i in range(0, np.size(contours)):
        x_sum = 0
        y_sum = 0
        contour_list = contours[i]
        for i in range(0, len(contour_list)):
            point = contour_list[i].tolist()
            x_sum +=  point[0][0]
            y_sum += point[0][1]
        means.append([x_sum/len(contour_list), y_sum/len(contour_list)])
    return(means)

def means_to_csv(means):
    file_name = f'means.csv'
    with open(file_name, 'w', newline="") as f:
        writer = csv.writer(f)
        for i in range(0, len(means)):
            data = means[i]
            writer.writerow(data)

def collinear(x1, y1, x2, y2, x3, y3):
    a = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)
    if a == 0:
        return True
    return False


def where_to_grab(num):
    contour_num = num
    contours = canny_draw_lines("shape-clipart.jpg")
    means = geometric_center(contours)
    contour = contours[contour_num]
    mean = means[contour_num]

    tip = False

    contour_list = contour.tolist()
    contour_x = []
    contour_y = []

    for i in range(len(contour_list)):
        contour_x.append(contour_list[i][0][0])
        contour_y.append(contour_list[i][0][1])
    points = []

    with tqdm(total = len(contour_list)-1, desc = "Finding points") as pbar:

        for i in range(0, len(contour_list)-1):
            tip = False
            data = contour_list[i]
            point = [data[0][0], data[0][1]]
            try:
                centerline_slope = (round(mean[1]) - point[1])/(round(mean[0]) - point[0])
            except ZeroDivisionError:
                centerline_slope = (round(mean[1])-point[1])/abs(round(mean[1]-point[1]))*1000000
            for x in range(min(contour_x), max(contour_x)+1):
                    y_calc = centerline_slope*(x-point[0]) + point[1]
                    # if tip:
                        # print(f"{x}, {y_calc}, centerline_slope: {centerline_slope}")
                    if [[x, round(y_calc)]] in contour_list:
                        # print(f"Orig pt: {point}, opp pt. : ({x}, {y_calc}),")
                        x_points = np.array([contour_x[i-1], contour_x[i], contour_x[i+1]])
                        y_points = np.array([contour_y[i-1], contour_y[i], contour_y[i+1]])
                        pts = np.vstack([x_points,np.ones(len(x_points))]).T
                        # Finding slope at original point
                        m,c = np.linalg.lstsq(pts,y_points,rcond=None)[0]
                        # least square has trouble with vertical line, define slope myself
                        if x_points[0]==x_points[1]==x_points[2]:
                            m = 1000000

                        # Setting centerline slope to a very small value if the slope is horizontal
                        # to avoid a divide by zero error.
                        if centerline_slope == 0:
                            centerline_slope = 0.001
                        if round(m, 2) == round(centerline_slope**-1, 2) or round(m**-1,2) == round(centerline_slope,2):
                            if [point, [x, round(y_calc)]] not in points:
                                points.append([point, [x, round(y_calc)]])
            pbar.update(1)

    for i, _ in enumerate(points):
        points[i][0][0] = points[i][0][0]*1/20
        points[i][0][1] = points[i][0][1]*1/20
        points[i][1][0] = points[i][1][0]*1/20
        points[i][1][1] = points[i][1][1]*1/20
    return points

means = geometric_center(canny_draw_lines('shape-clipart.jpg'))
means_to_csv(means)