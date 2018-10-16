import time

class Timer(object):
    enabled = True
    @staticmethod
    def enable(val=True):
        Timer.enabled = val
    def __init__(self,tag):
        self.tag = tag
        self.start = None
    def __enter__(self):
        self.start = time.time()
    def __exit__(self,a,b,c):
        if Timer.enabled:
            print ('[%s] Took %.2f seconds' % (self.tag, time.time() - self.start))

