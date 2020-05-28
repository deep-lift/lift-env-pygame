import pygame as pg

pg.init()

SCREEN_HEIGHT: int = 1000
SCREEN_WIDTH: int = 800
METER_PER_PIXEL: float = 30   # 미터당 픽셀
LIFT_WIDTH = 5
FONT = pg.font.Font('freesansbold.ttf', 16)
FIXED_TIME = 0.1