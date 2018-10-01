#!/usr/bin/python

import os

# ROS
import rospy
import rospkg
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2

# OPENCV
import cv2

# ROS-OPENCV
from cv_bridge import CvBridge

import numpy as np

# IMG PROC
from blob_detection import *
from background_subtraction import *
from optical_flow import *

# IMG Rectification
from image_rectification import Rectifier, BlockMatcher
from match import Matcher
from better_objecttracker import Object, ObjectTracker

from superpixel import SuperPixel, SuperPixelNode1, SuperPixelNode2, SuperPixelNode3
from sklearn.cluster import MeanShift
from sklearn.cluster import estimate_bandwidth

from stereo_utils import Timer, CameraReader, ROSCameraReader

from opencv_pipeline import Pipeline, SimpleNode

## Global Variables
opt = None
detector = BlobDetector()
bksub = BackgroundSubtractor()
bm = BlockMatcher(is_sgbm=True)

k_dilate = cv2.getStructuringElement(cv2.MORPH_DILATE, (7,7),(3,3))
k_erode = cv2.getStructuringElement(cv2.MORPH_ERODE, (3,3))
k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))

def neighborhood(x,y,n):
    #v = np.round(np.linspace(-n/2.,n/2.,n))
    v = np.linspace(-n/2.,n/2.,n,dtype=np.int)
    xs,ys = x+v,y+v
    return xs,ys

class ObjectManager(object):
    def __init__(self):
        self.current_index = 0
        self.objects = {} # dictionary of 'name, list of object'
    def add(self, new_object):
        if len(self.objects) > 0:
            cv2.imshow('object', self.objects.values()[-1][0].img)

        obj_found = False
        for k,v in self.objects.iteritems():
            l_v = len(v)
            for obj in np.random.choice(v,min(l_v,10)):
                if obj == new_object: #"same object"
                    v.append(new_object)
                    obj_found = True
                    break
            if obj_found:
                break
        else:
            new_name = 'object_{}'.format(self.current_index)
            print new_name
            self.objects[new_name] = [new_object]
            self.current_index += 1
            # no match

def projectDisparityTo3d(x,y,Q,d):
    x,y,z = (Q[0,0]*x + Q[0,3], Q[1,1]*y + Q[1,3], Q[2,3])
    w = Q[3,2]*d + Q[3,3]
    return (x/w, y/w, z/w)

def handle_disp(im_l, im_r, Q):
    return bm.apply(im_l, im_r)

def handle_opt(im):
    global opt
    if opt != None:
        opt_frame = opt.apply(im)
        opt_frame = cv2.cvtColor(opt_frame, cv2.COLOR_BGR2GRAY)
        #cv2.imshow("opt_flow", opt_frame)
        # activation should be at least greater than 30
        thr, opt_frame = cv2.threshold(opt_frame, 50, 255, cv2.THRESH_BINARY)
        return opt_frame #detector.apply(opt_frame), opt_frame
    else:
        opt = OpticalFlow(im)
        opt_frame = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        return opt_frame #([], opt_frame), opt_frame

def handle_bksub(im):
    mask = bksub.apply(im) 
    mask = cv2.dilate(mask, k_dilate, iterations=2)
    mask = cv2.GaussianBlur(mask,(3,3),0)
    mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,k_close)

    return mask #detector.apply(mask), mask

def fill_holes(mask):
    th, im_th = cv2.threshold(mask, 220, 255, cv2.THRESH_BINARY_INV);
    # Copy the thresholded image.
    im_floodfill = im_th.copy()
    # Mask used to flood filling.
    # Notice the size needs to be 2 pixels than the image.
    h, w = im_th.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)
    # Floodfill from point (0, 0)
    cv2.floodFill(im_floodfill, mask, (0,0), 255);
    # Invert floodfilled image
    im_floodfill_inv = cv2.bitwise_not(im_floodfill)
    # Combine the two images to get the foreground.
    filled = im_th | im_floodfill_inv
    return filled

def apply_superpixel(disp, ptss, target=None):
    frame = np.empty_like(disp)
    frame.fill(-16)
    for pts in ptss:
        l = disp[pts[0], pts[1]]
        valid_l = l[l>=0.0] # filter out invalid data
        valid_ratio = float(len(valid_l)) / len(l)
        if valid_ratio > 0.1 or len(valid_l) > 100:
            frame[pts] = np.mean(valid_l)
    return frame

def putText(frame, text, center):
    font = cv2.FONT_HERSHEY_SIMPLEX
    textSize = cv2.getTextSize(text,font,0.5,0)[0]
    pt = (int(center[0] - textSize[0]/2.0), int(center[1] - textSize[1]/2.0))
    cv2.putText(frame, text, pt, font,0.5,(255,255,255))

def apply_criteria(dist, frame, params):
    color = params['color']
    min_dist = params['min_dist']
    max_dist = params['max_dist']

    min_area = params['min_area']
    max_area = params['max_area']

    mask = np.zeros_like(dist, dtype=np.uint8)

    ## DISTANCE
    d_mask = cv2.inRange(dist, min_dist, max_dist)

    ## COLOR
    c_mask = np.zeros_like(dist, dtype=np.uint8)

    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    if color is 'red':
        mask_1 = cv2.inRange(hsv, (0,100,0), (20,255,255))
        mask_2 = cv2.inRange(hsv, (160,100,0), (180,255,255))
        cv2.add(mask_1,mask_2,dst = c_mask)
    elif color is 'blue': # 109 160 140
        cv2.inRange(hsv, (90,50,50), (130,255,255), dst=c_mask)
    elif color is 'green':
        cv2.inRange(hsv, (30,0,0), (80,255,255), dst=c_mask)
    elif color is 'yellow':
        cv2.inRange(hsv, (20,40,50), (30,255,255), dst=c_mask)

    cv2.bitwise_and(d_mask, c_mask, mask)

    ## AREA
    ctrs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    mask.fill(0)

    id_ctrs = []
    id_data = []

    for ctr in ctrs:
        m = cv2.moments(ctr)
        p_a = cv2.contourArea(ctr) # pixel coord area
        if m["m00"] > 0:
            cX = int(m["m10"] / m["m00"])
            cY = int(m["m01"] / m["m00"])
            d = dist[cY,cX]
            a = p_a * d * 1.5e-6
            if (min_area < a) and (a < max_area):
                id_ctrs.append(ctr)
                id_data.append((cX,cY,d,a,p_a))

    #Fill in Mask
    l = []
    for i in range(len(id_ctrs)):
        cv2.drawContours(mask, id_ctrs, i, color=255, thickness=-1)

    res = np.zeros_like(frame)
    idx = (mask == 255)
    res[idx] = frame[idx]

    font = cv2.FONT_HERSHEY_SIMPLEX

    # draw labels
    for dat in id_data:
        cX,cY,d,a,p_a = dat
        a *= 100 * 100 # convert to cm^2
        cv2.circle(res, (cX,cY), int(np.sqrt(p_a)), (255,255,255), thickness=2)
        putText(res, ('d: %.2f' % d), (cX,cY))
        putText(res, ('a: %.2f' % a), (cX,cY+20))

    return res, id_ctrs


mx,my = (0,0)
click = False

def mouse_cb(ev,x,y,flag,data):
    global mx,my, click
    if flag == 1:
        mx,my = x,y
        click = True

def setup_windows():
    cv2.namedWindow('sp_l')
    cv2.namedWindow('dist')
    cv2.namedWindow('filtered')

    cv2.setMouseCallback('sp_l', mouse_cb)

def toPointCloud(dist,frame):
    dist = dist.reshape(-1, 3)
    frame = frame.reshape(-1, 3).astype(np.float32)
    frame /= 255.0
    # assert same length?
    PointField = pcl2.PointField
    header = Header(frame_id='camera_link', stamp=rospy.Time.now())

    fields = []
    fields.append(PointField('x',0,PointField.FLOAT32,1))
    fields.append(PointField('y',4,PointField.FLOAT32,1))
    fields.append(PointField('z',8,PointField.FLOAT32,1))
    fields.append(PointField('r',12,PointField.FLOAT32,1))
    fields.append(PointField('g',16,PointField.FLOAT32,1))
    fields.append(PointField('b',20,PointField.FLOAT32,1))

    points = map(lambda ((x,y,z),(b,g,r)) : [z,-x,-y,r,g,b], zip(dist,frame))

    msg = pcl2.create_cloud(
            header,
            fields,
            points,
            )
    msg.height = 480
    msg.width = 640
    return msg

class DisparityNode(SimpleNode):
    def __init__(self):
        self.bm = BlockMatcher(is_sgbm=True)
        SimpleNode.__init__(self)
    def process(self,data):
        with Timer('disparity_node'):
            im_l, im_r, pts, sp_l = data 
            disp = self.bm.apply(im_l, im_r)
            disp = apply_superpixel(disp,pts)
        return sp_l, disp

def demo():
    global click
    Timer.enable(True)
    setup_windows()
    rospy.init_node('object_tracker')

    pcl = rospy.get_param('pcl', default=True)
    print 'pcl', pcl

    # initialize rectifier
    rospack = rospkg.RosPack()
    pkg_root = rospack.get_path('edwin')
    bridge = CvBridge()

    rectifier = Rectifier(
            param_l = os.path.join(pkg_root, 'Stereo/camera_info/left_camera.yaml'),
            param_r = os.path.join(pkg_root, 'Stereo/camera_info/right_camera.yaml')
            )

    obj_pub = rospy.Publisher('obj_point', PointStamped, queue_size=1)
    obj_msg = PointStamped()
    obj_msg.header.frame_id = 'camera_link'

    if pcl:
        pcl_pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=1)
        pcl_msg = PointCloud2()

    ### SETUP IMAGES
    dims = (480,640)
    dims3 = (480,640,3)

    left = np.empty(dims3, dtype=np.uint8)
    right = np.empty(dims3, dtype=np.uint8)
    im_l = np.empty(dims3,dtype=np.uint8)
    im_r = np.empty(dims3,dtype=np.uint8)
    sp_l = np.empty(dims3,dtype=np.uint8)

    superpixel = SuperPixel(800)

    nodes = [SuperPixelNode1(400), SuperPixelNode2(), SuperPixelNode3(), DisparityNode()]

    pipeline = Pipeline(nodes)

    with CameraReader(1) as cam_l, CameraReader(2) as cam_r:
        def input_fn():
            cam_l.read(left)
            cam_r.read(right)
            rectifier.apply(left, right, dst_l=im_l, dst_r=im_r)
            return left, right

        def output_fn(x):
            sp_l, disp = x

            cv2.imshow('sp_l', sp_l)
            cv2.imshow('disp', disp)

            raw_dist = cv2.reprojectImageTo3D((disp/16.).astype(np.float32), rectifier.Q, handleMissingValues=True)
            dist = raw_dist[:,:,2]
            if pcl:
                pcl_msg = toPointCloud(raw_dist, im_l)
                pcl_pub.publish(pcl_msg)
            cv2.imshow('dist', dist)

            params = {
                    'color' : 'yellow',
                    'min_dist' : 0.3,
                    'max_dist' : 1.5,
                    'min_area' : 0.003,
                    'max_area' : 1.0
                    }
            filtered, ctrs = apply_criteria(dist, sp_l, params)
            if len(ctrs) > 0:
                m = cv2.moments(ctrs[0]) # pull out biggest one--ish
                cX = int(m["m10"] / m["m00"])
                cY = int(m["m01"] / m["m00"])
                x,y,z = raw_dist[cY,cX]
                x,y,z = z,-x, -y
                obj_msg.header.stamp = rospy.Time.now()
                obj_msg.point.x = x
                obj_msg.point.y = y
                obj_msg.point.z = z
                obj_pub.publish(obj_msg)
            cv2.imshow('filtered', filtered)

        pipeline.run(input_fn, output_fn)

    #with CameraReader(1) as cam_l, CameraReader(2) as cam_r:
    ##with ROSCameraReader('/edwin_camera/left/image_raw') as cam_l, \
    ##        ROSCameraReader('/edwin_camera/right/image_raw') as cam_r:
    #    #matcher = Matcher()
    #    #tracker = ObjectTracker()
    #    #manager = ObjectManager()
    #    #tracker.set_target('medium','blue')

    #    while not rospy.is_shutdown():
    #        cam_l.read(left)
    #        cam_r.read(right)
    #        rectifier.apply(left, right, dst_l=im_l, dst_r=im_r)

    #        with Timer('SuperPixel'):
    #            superpixel.apply(im_l, dst=sp_l)

    #        cv2.imshow('sp_l', sp_l)

    #        with Timer('Disparity'):
    #            disp = handle_disp(im_l, im_r, rectifier.Q)
    #            #im_disp = cv2.normalize(disp,None,0.0,255.0,cv2.NORM_MINMAX).astype(np.uint8)
    #            #cv2.imshow('im_disp', im_disp)

    #        disp = apply_superpixel(disp, superpixel)

    #        raw_dist = cv2.reprojectImageTo3D((disp/16.).astype(np.float32), rectifier.Q, handleMissingValues=True)

    #        if pcl:
    #            with Timer('PCL'):
    #                pcl_msg = toPointCloud(raw_dist, im_l)
    #                pcl_pub.publish(pcl_msg)
    #        dist = raw_dist[:,:,2]

    #        cv2.imshow('dist', dist)

    #        #blur = cv2.GaussianBlur(im_l,(3,3),0) 
    #        #im_opt = handle_opt(blur)
    #        #im_bksub = handle_bksub(blur)

    #        identified = im_l.copy()
    #        params = {
    #                'color' : 'yellow',
    #                'min_dist' : 0.3,
    #                'max_dist' : 1.5,
    #                'min_area' : 0.003,
    #                'max_area' : 1.0
    #                }
    #        filtered, ctrs = apply_criteria(dist, sp_l, params)
    #        if len(ctrs) > 0:
    #            m = cv2.moments(ctrs[0]) # pull out biggest one--ish
    #            cX = int(m["m10"] / m["m00"])
    #            cY = int(m["m01"] / m["m00"])
    #            x,y,z = raw_dist[cY,cX]
    #            x,y,z = z,-x, -y
    #            obj_msg.header.stamp = rospy.Time.now()
    #            obj_msg.point.x = x
    #            obj_msg.point.y = y
    #            obj_msg.point.z = z
    #            obj_pub.publish(obj_msg)
    #        cv2.imshow('filtered', filtered)

    #        if cv2.waitKey(10) == 27:
    #            break

    cv2.destroyAllWindows()

def main():
    demo()

if __name__ == '__main__':
    main()
