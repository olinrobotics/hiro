from ctypes import *
import math
import random
import time
import cv2
import numpy as np

def sample(probs):
    s = sum(probs)
    probs = [a/s for a in probs]
    r = random.uniform(0, 1)
    for i in range(len(probs)):
        r = r - probs[i]
        if r <= 0:
            return i
    return len(probs)-1

def c_array(ctype, values):
    return (ctype * len(values))(*values)

class BOX(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("w", c_float),
                ("h", c_float)]

class IMAGE(Structure):
    _fields_ = [("w", c_int),
                ("h", c_int),
                ("c", c_int),
                ("data", POINTER(c_float))]

class METADATA(Structure):
    _fields_ = [("classes", c_int),
                ("names", POINTER(c_char_p))]

lib = CDLL("/home/lzuehsow/catkin_ws/src/darknet/libdarknet.so", RTLD_GLOBAL)

lib.network_width.argtypes = [c_void_p]
lib.network_width.restype = c_int
lib.network_height.argtypes = [c_void_p]
lib.network_height.restype = c_int

predict = lib.network_predict
predict.argtypes = [c_void_p, POINTER(c_float)]
predict.restype = POINTER(c_float)

make_boxes = lib.make_boxes
make_boxes.argtypes = [c_void_p]
make_boxes.restype = POINTER(BOX)

free_ptrs = lib.free_ptrs
free_ptrs.argtypes = [POINTER(c_void_p), c_int]

num_boxes = lib.num_boxes
num_boxes.argtypes = [c_void_p]
num_boxes.restype = c_int

make_probs = lib.make_probs
make_probs.argtypes = [c_void_p]
make_probs.restype = POINTER(POINTER(c_float))

detect = lib.network_predict
detect.argtypes = [c_void_p, IMAGE, c_float, c_float, c_float, POINTER(BOX), POINTER(POINTER(c_float))]

reset_rnn = lib.reset_rnn
reset_rnn.argtypes = [c_void_p]

load_net = lib.load_network
load_net.argtypes = [c_char_p, c_char_p, c_int]
load_net.restype = c_void_p

free_image = lib.free_image
free_image.argtypes = [IMAGE]

letterbox_image = lib.letterbox_image
letterbox_image.argtypes = [IMAGE, c_int, c_int]
letterbox_image.restype = IMAGE

load_meta = lib.get_metadata
lib.get_metadata.argtypes = [c_char_p]
lib.get_metadata.restype = METADATA

load_image = lib.load_image_color
load_image.argtypes = [c_char_p, c_int, c_int]
load_image.restype = IMAGE

predict_image = lib.network_predict_image
predict_image.argtypes = [c_void_p, IMAGE]
predict_image.restype = POINTER(c_float)

network_detect = lib.network_detect
network_detect.argtypes = [c_void_p, IMAGE, c_float, c_float, c_float, POINTER(BOX), POINTER(POINTER(c_float))]

def classify(net, meta, im):
    out = predict_image(net, im)
    res = []
    for i in range(meta.classes):
        res.append((meta.names[i], out[i]))
    res = sorted(res, key=lambda x: -x[1])
    return res

def detect(net, meta, image, thresh=.5, hier_thresh=.5, nms=.45):
    im = load_image(image, 0, 0)
    boxes = make_boxes(net)
    probs = make_probs(net)
    num =   num_boxes(net)

    network_detect(net, im, thresh, hier_thresh, nms, boxes, probs)

    res = []
    for j in range(num):
        for i in range(meta.classes):
            if probs[j][i] > 0:
                res.append((meta.names[i], probs[j][i], (boxes[j].x, boxes[j].y, boxes[j].w, boxes[j].h)))
    res = sorted(res, key=lambda x: -x[1])
    free_image(im)
    free_ptrs(cast(probs, POINTER(c_void_p)), num)
    return res[0]

def detect_img(fp):
    net = load_net((fp + "cfg/yolo.cfg"), (fp + "backup/yolo_200.weights"), 0)
    meta = load_meta(fp + "/data/toolpick.data")

    cap = cv2.VideoCapture(0)
    temp = True
    flag = 0 #Number of current frames- Keeping track of runtime
    maxruntime = 10000 #frames- Max runtime hard cutoff
    foundobj = False

    while(foundobj == False):
        #Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print ret
            print frame
            temp == False

        #Display the resulting frame

        # if foundobj is True:
        #     cv2.rectangle(frame, (int(math.floor(x)), int(math.floor(y))), (int(math.floor(x+w)), int(math.floor(x+h))), (255,0,0), 5)
        #     cv2.rectangle(frame, (0, 0), (5, 5), (255,0,0), 5)
        # tic = time.time()
        # if foundobj is True:
        #     if tic > (toc + 26):
        #         foundobj = False
        # cv2.imshow('frame', frame)

        # if foundobj is False and flag > 50:
        if flag > 50:
            print("Detecting...")
            tic = time.time()
            cv2.imwrite( fp + "temp.jpg", frame );
            # object_name, confidence, (x,y,w,h) = detect(net, meta, (fp + "data/34.jpg"))
            object_name, confidence, (x,y,w,h) = detect(net, meta, fp + "temp.jpg")
            toc = time.time()
            elapsed = (toc-tic)
            print('Took %.2f seconds' % elapsed)
            foundobj = True
            # print x, y, w, h
            return (x+(w/2), y+(h/2))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if flag > maxruntime:
            temp = cap.release() #if frame read correctly == True
            cv2.destroyAllWindows()

        flag = flag + 1

    #release the capture
    temp = cap.release() #if frame read correctly == True
    cv2.destroyAllWindows()


def yolo_detect():
    filepath = "/home/yichen/catkin_ws/src/darknet/"
    (x_coord, y_coord) = detect_img(filepath)

if __name__ == "__main__":

    main()
