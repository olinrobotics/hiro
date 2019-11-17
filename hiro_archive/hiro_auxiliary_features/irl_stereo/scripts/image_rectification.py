#!/usr/bin/python
import cv2
import numpy as np
import rospkg
import os

from yaml import load, Loader

def to_matrix(data):
    rows = data['rows']
    cols = data['cols']
    return np.reshape(data['data'], (rows,cols)).astype(np.float32)

def get_matrices(file_path):
    with open(file_path) as stream:
        data = load(stream, Loader=Loader)
        m = to_matrix(data['camera_matrix'])
        d = to_matrix(data['distortion_coefficients'])
        r = to_matrix(data['rectification_matrix'])
        p = to_matrix(data['projection_matrix'])
        return m, d, r, p

class Rectifier(object):
    def __init__(self, param_l, param_r):
        m_l, d_l, r_l, p_l = get_matrices(param_l)
        m_r, d_r, r_r, p_r = get_matrices(param_r)

        Q = np.zeros((4,4), dtype=np.float32)

        fx = p_l[0,0]
        cx = p_l[0,2]
        cy = p_l[1,2]
        fy = p_l[1,1]
        #Tx = p_l[0,3]
        Tx = p_r[0,3] / p_r[0,0]
        rcx = p_l[0,2]

        Q[0,0] = fy * Tx
        Q[0,3] = -fy * cx * Tx
        Q[1,1] = fx * Tx
        Q[1,3] = -fx * cy * Tx
        Q[2,3] = fx * fy * Tx
        Q[3,2] = -fy
        Q[3,3] = fy * (cx - rcx)

        self.Q = Q

        self.c_l_m1, self.c_l_m2 = cv2.initUndistortRectifyMap(m_l,d_l,r_l,p_l, (640,480), cv2.CV_32FC1)
        self.c_r_m1, self.c_r_m2 = cv2.initUndistortRectifyMap(m_r,d_r,r_r,p_r, (640,480), cv2.CV_32FC1)
    def apply(self, left, right, dst_l=None, dst_r=None):
        im_l = cv2.remap(left, self.c_l_m1, self.c_l_m2, cv2.INTER_LINEAR, dst=dst_l)
        im_r = cv2.remap(right, self.c_r_m1, self.c_r_m2, cv2.INTER_LINEAR, dst=dst_r)
        return im_l, im_r


class BlockMatcher(object):
    def __init__(self, is_sgbm=False):
        # fixed parameters for current camera
        # StereoSGBM([minDisparity, numDisparities, SADWindowSize[, P1[, P2[, disp12MaxDiff[, preFilterCap[, uniquenessRatio[, speckleWindowSize[, speckleRange[, fullDP]]]]]]]]]) -> <StereoSGBM object>
        if is_sgbm:
            self.bm = cv2.StereoSGBM(0, 128, 15, 200, 400, 0, 9 , 15, 100, 4, False)
        else:
            self.bm = cv2.StereoBM(cv2.STEREO_BM_BASIC_PRESET, 128, 45) #preset, ndisp, SADWindowSize
        self.is_sgbm = is_sgbm
    def apply(self, im_l, im_r):
        is_sgbm = self.is_sgbm

        if not is_sgbm:
            im_l =  cv2.cvtColor(im_l, cv2.COLOR_BGR2GRAY)
            im_r =  cv2.cvtColor(im_r, cv2.COLOR_BGR2GRAY)
        disp = self.bm.compute(im_l, im_r)
        return disp

if __name__ == "__main__":
    # Usage example
    rospack = rospkg.RosPack()
    pkg_root = rospack.get_path('edwin')

    cam_l = cv2.VideoCapture(2)
    cam_r = cv2.VideoCapture(1)

    rect = Rectifier(
            param_l = os.path.join(pkg_root, 'Stereo/camera_info/left_camera.yaml'),
            param_r = os.path.join(pkg_root, 'Stereo/camera_info/right_camera.yaml')
            )

    bm = BlockMatcher()

    while True:
        _, left = cam_l.read()
        _, right = cam_r.read()
        im_l, im_r = rect.apply(left, right)
        disp = bm.apply(im_l, im_r).astype(np.float32)

        #mn = disp.min()
        #mx = disp.max()
        #disp = (disp+mn) * (255 / (mx - mn))
        #disp = disp.astype(np.uint8)

        disp = cv2.normalize(disp, None, 0.0, 255.0, cv2.NORM_MINMAX).astype(np.uint8)

        cv2.imshow("left", im_l)
        cv2.imshow("right", im_r)
        cv2.imshow("disp", disp)

        cv2.waitKey(1)
