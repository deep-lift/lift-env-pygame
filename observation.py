
from render import *


class observation(object):

    def __init__(self):
        self.obsvector = []

    def reset(self):
        self.obsvector.clear()

    def add(self,value):
        self.obsvector.append(float(value));

    def add(self, vec:Vector3):
        self.obsvector.append(float(vec.x))
        self.obsvector.append(float(vec.y))
        self.obsvector.append(float(vec.z))

    def add(self,b:bool):
        if b:
            self.obsvector.append(1)
        else:
            self.obsvector.append(0)

