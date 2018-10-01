import cv2
import numpy as np
from abc import ABCMeta, abstractmethod
from threading import Thread, Lock
import Queue

from stereo_utils import CameraReader


p_lock = Lock()

class NukeOldDataQueue(Queue.Queue):
    def put(self,*args,**kwargs):
        if self.full():
            try:
                self.get(block=False)
            except Queue.Empty:
                pass
        Queue.Queue.put(self,*args,**kwargs)

class SimpleNode(Thread):
    __metaclass__ = ABCMeta
    def __init__(self):
        self.i_queue = NukeOldDataQueue(maxsize=10)
        self.o_queue = NukeOldDataQueue(maxsize=10)

        self.data = []
        self.i_lock = Lock()
        self.o_lock = Lock() # done lock
        self.s_lock = Lock() # stop lock

        self.stopped = False
        Thread.__init__(self)

    def run(self):

        self.s_lock.acquire()
        self.stopped = False
        self.s_lock.release()

        while True:

            ### CHECK STOPPED
            self.s_lock.acquire()
            stopped = self.stopped
            self.s_lock.release()
            if stopped:
                break

            self.i_lock.acquire()
            data = None
            try:
                data = self.i_queue.get(block=False)
            except Exception:
                pass
            self.i_lock.release()

            if data is not None:
                res = self.process(data)
                self.o_lock.acquire()
                self.o_queue.put(res,block=False)
                self.o_lock.release()

    @abstractmethod
    def process(self, data):
        pass

    def put(self, data):
        self.i_lock.acquire()
        self.i_queue.put(data,block=False)
        self.i_lock.release()

    def get(self):
        data = None

        self.o_lock.acquire()
        try:
            data = self.o_queue.get(block=False)
        except Exception:
            pass
        self.o_lock.release()
        
        return data

    def stop(self):
        self.s_lock.acquire()
        self.stopped = True
        self.s_lock.release()

class BlurNode(SimpleNode):
    def __init__(self):
        SimpleNode.__init__(self)
    def process(self,data):
        return cv2.GaussianBlur(data, (7,7), 0)

class ThreshNode(SimpleNode):
    def __init__(self):
        SimpleNode.__init__(self)
    def process(self,data):
        gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
        thresh, out = cv2.threshold(gray,100,200,cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        return out

class Pipeline(object):
    def __init__(self, nodes):
        self.nodes = nodes
    def run(self, input_fn, output_fn):
        nodes = self.nodes
        for n in nodes:
            n.start()
        n = len(nodes)

        while True:
            input_data = input_fn()
            nodes[0].put(input_data)

            for i in range(n): 
                o = nodes[i].get()
                if o is not None:
                    if (i+1 >= n): # output node
                        output_fn(o)
                    else: # intermediate node
                        nodes[i+1].put(o)

            k = cv2.waitKey(10)
            if k == 27: # ESC
                for n in nodes:
                    n.stop()
                return

def main():
    nodes = [BlurNode() for _ in range(4)]
    nodes.append(ThreshNode())
    p = Pipeline(nodes)

    with CameraReader(0) as cap:
        frame = np.empty((480,640,3), dtype=np.uint8)
        p.run(lambda: cap.read(), lambda o : cv2.imshow('output', o))

if __name__ == "__main__":
    main()











