import cv2
import numpy as np

from scipy import ndimage as ndi
from skimage.segmentation import slic

from stereo_utils import CameraReader, Timer

from skimage.future import graph
from skimage import data, segmentation, filters, color

from opencv_pipeline import SimpleNode

def weight_boundary(graph, src, dst, n):
    """
    Handle merging of nodes of a region boundary region adjacency graph.

    This function computes the `"weight"` and the count `"count"`
    attributes of the edge between `n` and the node formed after
    merging `src` and `dst`.


    Parameters
    ----------
    graph : RAG
        The graph under consideration.
    src, dst : int
        The vertices in `graph` to be merged.
    n : int
        A neighbor of `src` or `dst` or both.

    Returns
    -------
    data : dict
        A dictionary with the "weight" and "count" attributes to be
        assigned for the merged node.

    """
    default = {'weight': 0.0, 'count': 0}

    count_src = graph[src].get(n, default)['count']
    count_dst = graph[dst].get(n, default)['count']

    weight_src = graph[src].get(n, default)['weight']
    weight_dst = graph[dst].get(n, default)['weight']

    count = count_src + count_dst
    return {
        'count': count,
        'weight': (count_src * weight_src + count_dst * weight_dst)/count
    }


def merge_boundary(graph, src, dst):
    """Call back called before merging 2 nodes.

    In this case we don't need to do any computation here.
    """
    pass

class SuperPixel(object):
    def __init__(self, n_segments=100):
        dims = (480,640)
        dims3 = dims + (3,)
        #self.frame = np.empty(dims, dtype=np.uint8)
        self.superpixel = np.empty(dims3, dtype=np.uint8)
        self.n_segments = n_segments

        self.initialized = False
        self.rgb = None
        self.lab = None

        self.pts = []

    def get_segments(self, frame):

        if not self.initialized:
            self.rgb = np.empty_like(frame)
            self.lab = np.empty_like(frame)
            self.initialized = True

        #with Timer('super-labels1'):
        cv2.cvtColor(frame, cv2.COLOR_BGR2RGB,dst=self.rgb)
        labels = slic(self.rgb, self.n_segments)
        #with Timer('super-labels2'):
        edges = filters.sobel(color.rgb2gray(self.rgb))
        g = graph.rag_boundary(labels, edges)
        labels2 = graph.merge_hierarchical(labels, g, thresh=0.02, rag_copy=False,
                in_place_merge=True, merge_func=merge_boundary, weight_func=weight_boundary)
        return labels2.astype(np.uint8)

    def apply(self, frame, dst=None):
        #with Timer('super-segments'):
        segments = self.get_segments(frame)

        if dst is None:
            dst = self.superpixel
        with Timer('super-fill'):
            dst.fill(0)
            self.pts = []
            cv2.cvtColor(frame, cv2.COLOR_BGR2LAB, dst=self.lab)
            for i in range(1+np.max(segments)):
                pts = np.where(segments == i)
                self.pts.append(pts)

                ### "Better" Mean Color ...
                lab_t = np.average(self.lab[pts[0],pts[1]], axis=0).reshape((1,1,3)).astype(np.uint8)
                t = cv2.cvtColor(lab_t, cv2.COLOR_LAB2BGR).reshape(3)
                #t = np.average(frame[pts[0], pts[1]], axis=0).astype(np.uint8)
                ###
                dst[pts] = t
        return dst

class SuperPixelMatcher(object):
    def __init__(self, n_segments=100):
        dims = (480,640)
        dims3 = dims + (3,)
        self.superpixel = np.empty(dims3, dtype=np.uint8)
        self.tmp = np.empty(dims, dtype=np.uint8)
        self.n_segments = n_segments
        self.disp = np.empty(dims, dtype=np.uint8)

    def get_segments(self, frame):
        rgb = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        labels = slic(rgb, self.n_segments)
        edges = filters.sobel(color.rgb2gray(rgb))
        g = graph.rag_boundary(labels, edges)
        labels2 = graph.merge_hierarchical(labels, g, thresh=0.02, rag_copy=False,
                in_place_merge=True, merge_func=merge_boundary, weight_func=weight_boundary)
        return labels2.astype(np.uint8)

    def segment_data(self, frame, segments):
        data = []
        self.superpixel.fill(0)
        for i in range(1+np.max(segments)):
            pts = np.where(segments == i)
            col = np.average(frame[pts[0], pts[1]], axis=0).astype(np.uint8)

            #center of contour
            self.tmp.fill(0)
            self.tmp[pts] = 255
            c = cv2.findContours(self.tmp, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2][0]
            M = cv2.moments(c)

            if abs(M["m00"]) > 1e-3:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                self.superpixel[pts] = col

                data.append({
                    'idx' : i,
                    'size' : np.count_nonzero(pts),
                    'color' : col,
                    'center' : (cY,cX)
                    })
        return data

    def color_segments(self, frame, segments):
        self.superpixel.fill(0)
        for i in range(1+np.max(segments)):
            pts = np.where(segments == i)
            t = np.average(frame[pts[0], pts[1]], axis=0).astype(np.uint8)
            self.superpixel[pts] = t
        return self.superpixel.copy()

    def match(self,data_l, data_r,params):

        ds = abs(float(data_l['size'] - data_r['size'])/data_l['size'])

        cl = data_l['color'].reshape((1,1,3))
        cr = data_r['color'].reshape((1,1,3))

        cl = cv2.cvtColor(cl, cv2.COLOR_BGR2LAB)
        cr = cv2.cvtColor(cr, cv2.COLOR_BGR2LAB)
        orig_dc = np.linalg.norm(cl-cr)

        dy = abs(data_l['center'][0] - data_r['center'][0])
        dx = data_l['center'][1] - data_r['center'][1] # object should appear "more to the right" on the left camera

        cl_r = data_l['color'][2]
        cr_r = data_r['color'][2]
        dc = orig_dc < 500

        thresh = sum(params)
        ss,sc,sy,sx = (0.3, 500.0, 15.0, 50.)
        ks,kc,ky,kx = params

        ds /= ss
        dc /= sc
        dy /= sy
        dx /= sx

        succ = ds<ks and dc<kc and dy<ky and dx < kx and dx > 0 \
                and data_l['center'][1] > sx and data_r['center'][1] < 640 - sx
        score = thresh - ks*ds - kc*dc - ky*dy - kx*dx
        return succ, score

    def apply(self, left, right):
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        #cv2.putText(img,'OpenCV',(10,500), font, 4,(255,255,255),2,cv2.LINE_AA)

        # return "disparity image"
        seg_l = self.get_segments(left)
        seg_r = self.get_segments(right)

        pxl_l = self.color_segments(left, seg_l)
        pxl_r = self.color_segments(right, seg_r)

        seg_data_l = self.segment_data(left, seg_l)
        seg_data_r = self.segment_data(right, seg_r)
        cnt = 0

        def swap_axes(a):
            return (a[1],a[0])

        #c[0] = y
        #c[1] = x
        #Point(x,y)

        ks = 1.0
        kc = 2.0
        ky = 5.0
        kx = 2.0
        params = (ks,kc,ky,kx)
        self.disp.fill(0)
        for s_l in seg_data_l:
            best_r = None
            best_score = -99999

            ## MATCHING WITH RIGHT IMAGE
            for s_r in seg_data_r:
                succ, score = self.match(s_l, s_r, params) 
                if succ and score > best_score:
                    best_score = score
                    best_r = s_r

            if best_r is not None:
                s_r = best_r
                disparity = s_l['center'][1] - s_r['center'][1]
                self.disp[seg_l == s_l['idx']] = disparity
                c_l = swap_axes(s_l['center'])
                c_r = swap_axes(s_r['center'])

                cv2.putText(pxl_l, ('%d' % s_l['idx']), c_l, font, 0.5, (255,255,255))
                cv2.putText(pxl_r, ('%d' % s_l['idx']), c_r, font, 0.5, (255,255,255))

                cv2.circle(pxl_l, c_l, int(10), np.subtract((255,255,255), s_l['color']))
                cv2.circle(pxl_r, c_r, int(10), np.subtract((255,255,255), s_r['color']))

        return pxl_l, pxl_r, self.disp
        # arrange by color, size, and center



def test_matcher():
    with CameraReader(1) as cap_l, CameraReader(2) as cap_r:
        while True:
            m = SuperPixelMatcher(400)

            dims = (480,640)
            dims3 = dims + (3,)

            frame_l = np.empty(dims3, dtype=np.uint8)
            frame_r = np.empty(dims3, dtype=np.uint8)

            with Timer('Cap'):
                cap_l.read(frame_l)
                cap_r.read(frame_r)
            with Timer('SuperPixel'):
                pxl_l, pxl_r, disp = m.apply(frame_l, frame_r)

            cv2.imshow('l', pxl_l)
            cv2.imshow('r', pxl_r)
            cv2.imshow('d', disp)

            if cv2.waitKey(100) == 27:
                break


def main():
    Timer.enable()

    with CameraReader(1) as cap:
        k_dilate = cv2.getStructuringElement(cv2.MORPH_DILATE, (5,5),(3,3))

        dims = (480,640)
        dims3 = dims + (3,)

        frame = np.empty(dims3, dtype=np.uint8)
        thresh = np.empty(dims, dtype=np.uint8)
        blur = np.empty(dims3, dtype=np.uint8)
        hsv = np.empty(dims3, dtype=np.uint8)
        gray = np.empty(dims, dtype=np.uint8)
        segments = np.empty(dims, dtype=np.uint8)
        superpixel = np.empty(dims3, dtype=np.uint8)
        shifted = np.empty(dims3, dtype=np.float32)

        s = SuperPixel(200)

        # setup matrices
        while True:
            with Timer('Cap'):
                cap.read(frame)
            with Timer('SuperPixel'):
                superpixel = s.apply(frame)

            cv2.imshow('frame', frame)
            cv2.imshow('slic', superpixel)

            if cv2.waitKey(10) == 27: # ESC
                break

if __name__ == "__main__":
    #test_matcher()
    main()



######################################
# SUPERPIXEL PIPELINE IMPLEMENTATION #
######################################

class SuperPixelNode1(SimpleNode):
    def __init__(self, n_segments=100):
        self.n_segments = n_segments
        self.rgb = None
        SimpleNode.__init__(self)
    def process(self, data):
        with Timer('superpixelnode_1'):
            im_l, im_r = data
            if self.rgb is None:
                self.rgb = np.empty_like(im_l)
            cv2.cvtColor(im_l, cv2.COLOR_BGR2RGB,dst=self.rgb)
            labels = slic(self.rgb, self.n_segments)
        return im_l, im_r, self.rgb, labels

class SuperPixelNode2(SimpleNode):
    def __init__(self):
        SimpleNode.__init__(self)
    def process(self, data):
        with Timer('superpixelnode_2'):
            im_l, im_r, rgb, labels = data
            edges = filters.sobel(color.rgb2gray(rgb))
            g = graph.rag_boundary(labels, edges)
            labels2 = graph.merge_hierarchical(labels, g, thresh=0.02, rag_copy=False,
                    in_place_merge=True, merge_func=merge_boundary, weight_func=weight_boundary)
        return im_l, im_r, labels2.astype(np.uint8)

class SuperPixelNode3(SimpleNode):
    def __init__(self):
        self.lab = None
        SimpleNode.__init__(self)

    def process(self, data):
        with Timer('superpixelnode_3'):
            im_l, im_r, segments = data

            if self.lab is None:
                self.lab = np.empty_like(im_l)
            sp_img = np.zeros_like(im_l)
            ptss = []

            cv2.cvtColor(im_l, cv2.COLOR_BGR2LAB, dst=self.lab)
            for i in range(1+np.max(segments)):
                pts = np.where(segments == i)
                ptss.append(pts)
                lab_t = np.average(self.lab[pts[0],pts[1]], axis=0).reshape((1,1,3)).astype(np.uint8)
                t = cv2.cvtColor(lab_t, cv2.COLOR_LAB2BGR).reshape(3)
                sp_img[pts] = t
        return im_l, im_r, ptss, sp_img
################
