import math
import pygame as pg

pg.init()

SCREEN_HEIGHT: int = 800
SCREEN_WIDTH: int = 600
METER_PER_PIXEL: float = 30   # 미터당 픽셀
LIFT_WIDTH = 5
FONT = pg.font.Font('freesansbold.ttf', 16)

class Vector3:
    x: float
    y: float
    z: float

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "(" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + ")"

    def get_length(self):
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def __add__(self, v):
        return Vector3(self.x + v.x, self.y + v.y, self.z + v.z)

    def __sub__(self, v):
        return Vector3(self.x - v.x, self.y - v.y, self.z - v.z)

    def __mul__(self, n):
        return Vector3(self.x * n, self.y * n, self.z * n)

    @staticmethod
    def dot_product(v1, v2):
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z


def DrawText(scr, x, y, str, color):
    text = FONT.render(str, True, color)
    textRect = text.get_rect()
    textRect.top = y
    textRect.left = x
    scr.blit(text, textRect)


def ToScreenPos(pos):
    x: int = pos.x * METER_PER_PIXEL
    y: int = SCREEN_HEIGHT-(pos.y * METER_PER_PIXEL)
    return x, y


def ToGamePos(x,y):
    return Vector3(x / METER_PER_PIXEL, (SCREEN_HEIGHT - y) / METER_PER_PIXEL, 0)


BLACK = (0,  0,  0)
WHITE = (255, 255, 255)
BLUE = (0,  0, 255)
GREEN = (0, 255,  0)
RED = (255,  0,  0)
GRAY = (128, 128, 128)


class passenger_spawn(object):
    def __init__(self):
        self.step =0
        self.floor =-1
        destfloor = []

