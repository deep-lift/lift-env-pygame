
from render import *


class observation(object):

    def __init__(self):
        self.obsvector = []

    def reset(self):
        self.obsvector.clear()

    def add(self,value:float):
        self.obsvector.append(float(value));

    def add(self, vec:Vector3):
        self.obsvector.append(float(vec.x))
        self.obsvector.append(float(vec.y))
        self.obsvector.append(float(vec.z))

    def add(self,value):
        self.obsvector.append(value)

