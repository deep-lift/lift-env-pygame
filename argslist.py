import pygame as pg
pg.init()

# ===================== ENV =======================
FONT = pg.font.Font('freesansbold.ttf', 16)
SCREEN_HEIGHT: int = 800
SCREEN_WIDTH: int = 600
METER_PER_PIXEL: float = 20.779
LIFT_WIDTH = 5.775
FIXED_TIME = 0.1
GAME_SPEED = 1000
RENDER = False
N_PASSENGER = 3
IS_EVENT_DRIVEN = True

# ===================== AGENT =====================
N_AGENTS = 1
N_OBSERVATION = 15
N_STATE = 31
N_ACTION = 10
N_MAX_STEPS = 1000

# N_OBSERVATION = 4
# N_STATE = 0
# N_ACTION = 2
# N_MAX_STEPS = 200

N_HIDDEN = 64
REPLAY_MEM = 50000

PRINT_INTERVAL = 100
BATCH_SIZE = 128
GAMMA = 0.99

# SAVE_INTERVAL = 1000
# START_TRAIN_EPISODE = 10
# TARGET_UPDATE = 10