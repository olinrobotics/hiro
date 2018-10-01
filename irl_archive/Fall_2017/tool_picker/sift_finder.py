import numpy as np
import cv2
import matplotlib.pyplot as plt
import sys
import math

def remove_outliers(data):
    # data is a list of 2D arrays
    total = [0,0]
    for point in data:
        total[0] += point[0][0]
        total[1] += point[0][1]
    mean = [total[0]/len(data), total[1]/len(data)]
    dist = []
    for point in data:
        x = point[0][0]
        y = point[0][1]
        dist.append([math.sqrt((x-mean[0])**2 + (y-mean[1])**2)])
    dist_mean = np.median(dist)
    dist_std = np.std(dist)
    i = 0
    while i < len(data):
        if dist_mean - 1.5*dist_std > dist[i] or dist_mean + 1.5*dist_std < dist[i]:
            dist = np.delete(dist,i)
            data = np.delete(data,i,0)
        else:
            i += 1
    return data

def sift_finder(camera_img, tool_id):
    MIN_MATCH_COUNT = 5
    sift = cv2.xfeatures2d.SIFT_create(contrastThreshold=0.04)

    # works: cutter, clamp, wrench, piler, screwdriver, scissors
    img_path = tool_id + '.png'
    img1 = cv2.imread(img_path)
    gray1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
    kp1, des1 = sift.detectAndCompute(gray1,None)

    img2 = camera_img
    # img2 = cv2.imread('usb_cam1.png')
    gray2= cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
    kp2, des2 = sift.detectAndCompute(gray2,None)
    # img2 = cv2.drawKeypoints(gray2,kp2,img2)
    # cv2.namedWindow('image2',cv2.WINDOW_NORMAL)
    # cv2.resizeWindow('image2', 800,800)
    # cv2.imshow('image2',img2)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(des1,des2,k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.69*n.distance:
            good.append(m)

    if len(good) >= MIN_MATCH_COUNT:
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = remove_outliers(dst_pts)
        sum_x = 0
        sum_y = 0
        for point in dst_pts:
            sum_x += point[0][0]
            sum_y += point[0][1]
            img2 = cv2.circle(img2, (int(point[0][0]),int(point[0][1])),10, (0,255,0))
        avg_x = sum_x / len(dst_pts)
        avg_y = sum_y / len(dst_pts)

        # ''' Visualizer for detection points '''
        # img3 = cv2.circle(img2, (int(avg_x),int(avg_y)),10, (255,0,0))
        # cv2.imshow('result', img3)
        #
        # cv2.waitKey(0)
        return([True, (avg_x, avg_y)])
    else:
        print(len(good))
        return([False, (0,0)])

    '''Visualizer for match with original pictures'''
    # if len(good)>MIN_MATCH_COUNT:
    #     src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    #     dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
    #
    #     M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
    #     matchesMask = mask.ravel().tolist()
    #     h,w = gray1.shape
    #     pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    #     dst = cv2.perspectiveTransform(pts,M)
    #
    #     img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
    #
    # else:
    #     print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
    #     matchesMask = None
    #
    # draw_params = dict(matchColor = (0,255,255), # draw matches in green color
    #                    singlePointColor = None,
    #                    matchesMask = matchesMask, # draw only inliers
    #                    flags = 2)
    #
    # img3 = cv2.drawMatches(gray1,kp1,gray2,kp2,good,None,**draw_params)
    #
    # cv2.namedWindow('image3',cv2.WINDOW_NORMAL)
    # cv2.resizeWindow('image3', 800,800)
    # cv2.imshow('image3',img3)
    #
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

if __name__ == "__main__":
    [flag, coords] = sift_finder('sa','Piler')
    print(flag)
    print(coords)
