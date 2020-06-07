import pygame as pg

pg.init()

SCREEN_HEIGHT: int = 800
SCREEN_WIDTH: int = 600

METER_PER_PIXEL: float = 20.779
LIFT_WIDTH = 5.775

FONT = pg.font.Font('freesansbold.ttf', 16)
FIXED_TIME = 0.1

N_AGENTS = 4
N_OBSERVATION = 15
N_STATE = 31
N_ACTION = 3

N_MAX_STEPS = 2000

GAME_SPEED = 1000000

RENDER = True
