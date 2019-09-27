from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse
import random as rng
import roslib
import rospy
import pyrealsense2 as rs
import logging


def test():
    # Configure depth and color streams...
    # ...from Camera 1
    pipeline_1 = rs.pipeline()
    config_1 = rs.config()
    config_1.enable_device('802212060621')
    config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming from both cameras
    pipeline_1.start(config_1)
    img_counter = 0
    try:
        while True:

            # Camera 1
            # Wait for a coherent pair of frames: depth and color
            frames_1 = pipeline_1.wait_for_frames()
            color_frame_1 = frames_1.get_color_frame()

            #depth_frame_1 or
            if not color_frame_1:
                continue
            # Convert images to numpy arrays
            color_image_1 = np.uint8(color_frame_1.get_data())

            images = np.hstack((color_image_1))
            #depth_colormap_1))
            color_image_hsv = cv.cvtColor(color_image_1, cv.COLOR_BGR2HSV)
            # Show images from both cameras
            cv.namedWindow('RealSense', cv.WINDOW_NORMAL)
            cv.imshow('RealSense', color_image_1)
            cv.waitKey(1)

            ch = cv.waitKey(25)
                
            if ch%256 == 32:
                # SPACE pressed, take picture, exit loop
                img_name = "opencv_frame_{}.jpeg".format(img_counter)
                cv.imwrite(img_name, color_image_1)
                print("{} written!".format(img_name))
                img_counter += 1
                print("Escape hit, closing...")
                break

    finally:
        # Stop streaming

        def color_range():
            #define ranges to block out certain colors
            white = np.uint8([[[173,173,173]]])
            hsvWhite = cv.cvtColor(white, cv.COLOR_BGR2HSV)
            lowerLimitWhite = np.uint8([hsvWhite[0][0][0]-255,100,100])
            upperLimitWhite = np.uint8([hsvWhite[0][0][0]+10,255,210])

            black = np.uint8([[[160,200,230]]])
            hsvBlack = cv.cvtColor(black, cv.COLOR_BGR2HSV)
            lowerLimitBlack = np.uint8([hsvBlack[0][0][0]-10,130,100])
            upperLimitBlack = np.uint8([hsvBlack[0][0][0]+225,255,200])

            return lowerLimitWhite, upperLimitWhite,lowerLimitBlack, upperLimitBlack

        def color_detect(threshold, lowerR, upperR, lowerT, upperT):
            #Convert RGB to HSV values, create a mask over original image only showing those colors
            hsv = cv.cvtColor(src, cv.COLOR_BGR2HSV)

            RedOctagon = []
            BlueSquare = []

            mask1 = cv.inRange(hsv, lowerR, upperR)
            res1 = cv.bitwise_and(color_image_1, color_image_1, mask=mask1)
            res1 = cv.medianBlur(res1, 5)
            RedColor = cv.imshow('Color Detector 1', res1)

            mask2 = cv.inRange(hsv, lowerT, upperT)
            res2 = cv.bitwise_and(color_image_1, color_image_1, mask=mask2)
            res2 = cv.medianBlur(res2, 5)
            TealColorImg = cv.imshow('Color Detector 2', res2)

            cv.imwrite("Red_Color.jpeg", res1)
            cv.imwrite("Blue_Color.jpeg", res2)

            src2 = cv.imread("Red_Color.jpeg")
            src3 = cv.imread("Blue_Color.jpeg")
            if src2 is None or src3 is None:
                print('Could not open or find the image:', args2.input)
                exit(0)

            #Get contours of objects from colored images
            RedColorImgGray = cv.cvtColor(src2, cv.COLOR_BGR2GRAY)
            RedColorImgGray = cv.blur(RedColorImgGray, (3,3))

            BlueColorImgGray = cv.cvtColor(src3, cv.COLOR_BGR2GRAY)
            BlueColorImgGray = cv.blur(BlueColorImgGray, (3,3))

            canny_output2 = cv.Canny(RedColorImgGray, threshold, threshold * 2)
            canny_output3 = cv.Canny(BlueColorImgGray, threshold, threshold * 2)

            _, contours2, _ = cv.findContours(canny_output2, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            _, contours3, _ = cv.findContours(canny_output3, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            contours_poly2 = [None]*len(contours2)
            contours_poly3 = [None]*len(contours3)

            for i, c in enumerate(contours2):
                contours_poly2[i] = cv.approxPolyDP(c, 3, True)

            drawingColor = np.zeros((canny_output2.shape[0], canny_output2.shape[1], 3), dtype=np.uint8)

            for i in range(len(contours2)):
                color = (rng.randint(0,83), rng.randint(83,150), rng.randint(150,256))
                cv.drawContours(drawingColor, contours_poly2, i, color)

            for cnt in contours2:
                M = cv.moments(cnt)
                approx = cv.approxPolyDP(cnt,0.01*cv.arcLength(cnt,True),True)

                #Get x,y coordinates for red octagons
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv.circle(drawingColor, (cX, cY), 1, (255, 255, 255), -1)
                    RedOctagon.append(['Red', cX, cY])

            for i, c in enumerate(contours3):
                contours_poly3[i] = cv.approxPolyDP(c, 3, True)

            drawingColor2 = np.zeros((canny_output3.shape[0], canny_output3.shape[1], 3), dtype=np.uint8)

            for i in range(len(contours3)):
                color = (rng.randint(0,83), rng.randint(83,150), rng.randint(150,256))
                cv.drawContours(drawingColor2, contours_poly3, i, color)

            for cnt in contours3:
                M = cv.moments(cnt)
                approx = cv.approxPolyDP(cnt,0.01*cv.arcLength(cnt,True),True)

                #Get x,y coordinates for blue squares
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv.circle(drawingColor2, (cX, cY), 2, (255, 255, 255), -1)
                    BlueSquare.append(['Blue', cX, cY])

            cv.imshow('Contours Mask', drawingColor)
            cv.imshow('Contours Mask2', drawingColor2)
            return RedOctagon, BlueSquare

        def thresh_callback(threshold):
            colorRange = color_range()
            #Detect the colors in the range of colors 
            colorImg = color_detect(threshold, colorRange[0], colorRange[1], colorRange[2], colorRange[3])
            return colorImg

        def Grid(thresh):
            #8 x 8 grid of the chessboard
            colors = thresh_callback(thresh)

            board = []
            grid = []

            A = [17,70]
            B = [70,130]
            C = [132,185]
            D = [185,246]
            E = [246,300]
            F = [300,359]
            G = [359,413]
            H = [413,472]
            letters = [A,B,C,D,E,F,G,H]

            one = [81,138]
            two = [138,197]
            three = [197,253]
            four = [253,308]
            five = [308,366]
            six = [366,423]
            seven = [423,478]
            eight = [478,534]
            numbers = [one,two,three,four,five,six,seven,eight]

            board = [[0]*8 for i in range(8)]

            #Checks each square on the chess board for an object centerpoint
            for l in range(len(letters)):
                for n in range(len(numbers)):
                    for col in range(len(colors)):
                        for color in range(len(colors[col])):
                            if letters[l][0] < colors[col][color][2] < letters[l][1]:
                                if numbers[n][0] < colors[col][color][1] < numbers[n][1]:
                                    if colors[col][color][0] == 'Red':
                                        if board[l][n] == 0:
                                            board[l][n] = 1
                                    if colors[col][color][0] == 'Blue':
                                        if board[l][n] == 0: 
                                            board[l][n] = 2
            return board

        parser = argparse.ArgumentParser(description='Code for recognizing squares and octagons.')
        parser.add_argument('--input', help='Path to input image.', default='opencv_frame_{}.jpeg'.format(img_counter-1))
        args = parser.parse_args()
        src = cv.imread(args.input)
        if src is None:
            print('Could not open or find the image:', args.input)
            exit(0)
        # Convert image to gray and blur it
        src_gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
        src_gray = cv.blur(src_gray, (3,3))
        source_window = 'Source'
        cv.namedWindow(source_window)
        cv.imshow(source_window, src)
        thresh = 40
        grid = Grid(thresh)

        #cv.waitKey()

        cv.destroyAllWindows()

        pipeline_1.stop()

        print(grid)
    
    return grid

if __name__ == "__main__":
    test()