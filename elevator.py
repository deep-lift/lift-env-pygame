

import building
import pygame as pg
import simpy
import sys

class env_time(object):

    def __init__(self,fixedtime):
        env_time.fixedtime= fixedtime
        env_time.currenttick = 0;

  



class elevator_env(object):
    elevator_count:int     = 4 
    floors       :int     = 10
    passenger    :int     = 100
    height       :float   = 3.5
    speed        :float   = 3  
    decelerate   :float   = 1
    acelerate    :float   = 1
    open         :float   = 1
    close        :float   = 1 
    turn         :float   = 1
    capacity     :int     = 15
    actionTofloor:int     = 0
    fixedTime    :float   = 0.1
    maxstep      :int     = 50000
    bd           :building
    heuristic    :bool    = True
              
    


    def __init__(self,size_x:int,size_y:int):
        pg.init()
        self.screen = pg.display.set_mode([size_x,size_y],pg.DOUBLEBUF)
        self.display = pg.display.set_caption("elevator")
        self.clock = pg.time.Clock()
        self.init_env()

    def init_env(self):
        self.bd =  building.cbuilding(self,self.screen)

    #def step(self,act:dict):
       # self.bd.

    def step(self):
       # for event in pg.event.get():
	   #	    if event.type == pg.QUIT: sys.exit()
        #self.clock.tick(1000)
        self.bd.update_step()
        self.bd.render()
        pg.display.update()
        






