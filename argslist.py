import pygame as pg

pg.init()

SCREEN_HEIGHT: int = 800
SCREEN_WIDTH: int = 600

#엘베 갯수와 층수에 맞게 상대적으로 셋팅시에 변경되어야 할 값인데
#값이 변하질 않음 그래서 우선은 값을 계산해서 입력해준다..
METER_PER_PIXEL: float = 20.779   # 미터당 픽셀
LIFT_WIDTH = 5.775

FONT = pg.font.Font('freesansbold.ttf', 16)
FIXED_TIME = 0.1