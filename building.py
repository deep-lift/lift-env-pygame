import random
import fsm
import math
import elevator
from enum import Enum

import pygame as pg

BLACK= ( 0,  0,  0)
WHITE= (255,255,255)
BLUE = ( 0,  0,255)
GREEN= ( 0,255,  0)
RED  = (255,  0,  0)
GRAY = (128,128,128)


FIXED_TIME:float;
SCREEN_HEIGHT:int
METERPERPIXEL:float;   #미터당 픽셀


def ToScreenPos(pos):
    x:int = pos.x*METERPERPIXEL
    y:int = SCREEN_HEIGHT-(pos.y*METERPERPIXEL)
    

    return x,y

def ToGamePos(x,y):
    return Vector3(x/METERPERPIXEL,(SCREEN_HEIGHT-y)/METERPERPIXEL,0)
    #return Vector3(x/METERPERPIXEL,(y)/METERPERPIXEL,0)


    
def DrawText(x,y,str,color):
    text = FONT.render(str, True, color)
    textRect = text.get_rect()  
    textRect.top  = y
    textRect.left = x
    SCREEN.blit(text, textRect) 


class MoveState(Enum):
    stop =  0;
    down =  1;
    up   =  2;

class Vector3:
    x:float
    y:float
    z:float

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
  # Used for debugging. This method is called when you print an instance  
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
    
  # def __div__(self, n):
  #   n /= -1
  #   return self * n
    
    @staticmethod
    def dot_product(v1, v2):
        return (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z)



class clift(object):
    """description of class"""
    #_building      : cbuilding
    #car_no         : int
    #max_speed      : float
    #curr_speed     : float 
    #state          : fsm.State
    #curr_floor     : float
    #req_floor      : dict  = {MoveState.stop:-1,MoveState.down:-1,MoveState.up:-1 }
    movedir:list = [0,-1,1]

    #hieght         : float
    #move           : MoveState = MoveState.stop
    #next_event     : fsm.Event
    #next_transition_time :float = 0
    #act_fsm        : fsm.cfsm
    #
    #pos            : Vector3
    #action          = None;
    #reqdecision_floor      : int
    #requestDecision:bool = False
    #
    #cool_time: float
    #
    #
    #
    #verticals      :list = []

    

    def __init__(self,no:int,bd:object):
        self.car_no = no;
        self._building = bd
        self.max_speed = bd._env.speed
        self.curr_speed = 0
        self.hieght = bd._env.height
        self.act_fsm = fsm.cfsm(fsm.State.Ready)
        self.pos = Vector3(0,0,0);

        self.passengers     : list = []   
        self.move = MoveState.stop
        
        

        self.act_fsm.add_transition(fsm.State.Ready, fsm.Event.Call, fsm.State.Accelate);
        #self.act_fsm.add_transition(fsm.State.Ready, fsm.Event.Arrived, fsm.State.DoorOpening);
        self.act_fsm.add_transition(fsm.State.Ready, fsm.Event.DoorOpenRequest, fsm.State.DoorOpening);
        
        self.act_fsm.add_transition(fsm.State.Accelate, fsm.Event.AccelateEnd, fsm.State.NormalMove);
        self.act_fsm.add_transition(fsm.State.Accelate, fsm.Event.DecelerateStart, fsm.State.NormalMove);
       
        
        self.act_fsm.add_transition(fsm.State.NormalMove, fsm.Event.DecelerateStart, fsm.State.Decelerate);
      
        
        self.act_fsm.add_transition(fsm.State.Decelerate, fsm.Event.Arrived, fsm.State.MoveStop);
        
        self.act_fsm.add_transition(fsm.State.MoveStop, fsm.Event.DoorOpenRequest, fsm.State.DoorOpening);
        self.act_fsm.add_transition(fsm.State.MoveStop, fsm.Event.EmptyPassenger, fsm.State.Ready);
        self.act_fsm.add_transition(fsm.State.MoveStop, fsm.Event.DoorCloseEnd, fsm.State.Accelate);
        
        self.act_fsm.add_transition(fsm.State.DoorOpening, fsm.Event.DoorOpenEnd, fsm.State.DoorOpened);

        self.act_fsm.add_transition(fsm.State.DoorOpened, fsm.Event.DoorCloseStart, fsm.State.DoorClosing);
        self.act_fsm.add_transition(fsm.State.DoorClosing, fsm.Event.DoorCloseEnd, fsm.State.Accelate);
        self.act_fsm.add_transition(fsm.State.DoorClosing, fsm.Event.EmptyPassenger, fsm.State.Ready);

       
        self.elevatorAction = dict();

        self.elevatorAction[fsm.State.Ready] = self.act_ready;                #문닫고 멈춰있는 상태
        self.elevatorAction[fsm.State.NormalMove] = self.act_normalmove;      #위아래 어느쪽이든 정상적인 이동상태
        self.elevatorAction[fsm.State.Decelerate] = self.act_decelerate;      #다음층에 멈추기 위한 감속상태
        self.elevatorAction[fsm.State.MoveStop] = self.act_movestop;          #멈춘상태
        self.elevatorAction[fsm.State.DoorOpening] = self.act_dooropening;    #문열는중
        self.elevatorAction[fsm.State.DoorOpened] = self.act_dooropened;      #문열린 상태에서 승객내리고 타고
        self.elevatorAction[fsm.State.DoorClosing] = self.act_doorclosing;    #문닫히는 동안.
        self.elevatorAction[fsm.State.Accelate] = self.act_accelate;          #이동에 대한 가속상태
        self.elevatorAction[fsm.State.Turn] = self.act_turn;

    def Init(self,no:int,start_floor:int):
        self.curr_floor = start_floor;

     
        self.pos.x = LIFT_WIDTH*(no+1)
        self.pos.y = self.hieght*(start_floor+1)
        self.move = MoveState.stop
        self.passengers.clear()

        self.reqState = fsm.State.End
        self.reqfloor = -1
        self.reqTime  = 0
        self.requestDecision = False
        self.cool_time = 0;
        self.reqdecision_floor = -1
        self.next_transition_time =0
        self.next_event = fsm.State.End

        self.req_floor =  {MoveState.stop:-1,MoveState.down:-1,MoveState.up:-1 }

        self.verticals = []
        for i in range(elevator.elevator_env.floors):
            vline = verticalLine(i,self.pos.x)
            self.verticals.append(vline);


    def is_enterable(self):
        return True


    def set_transition_delay(self,event:fsm.Event,delay:float,add:bool = False):

        if add == True:
            self.next_transition_time += delay;
        else :
            self.next_transition_time = delay;

        self.next_event = event;

    def chk_transtime(self):
 
        if  self.next_event == fsm.Event.End and self.next_transition_time==0 :
            return True;
    
        self.next_transition_time -= FIXED_TIME;

        if self.next_transition_time  > 0:
            return False;
        
        self.act_fsm.transition(self.next_event)
        self.next_event = fsm.Event.End
        self.next_transition_time = 0

        return True;

    def chk_floor(self):
        floor =-1
        next_floor =-1
        floor,next_floor = self.get_nextfloor();

        

        if int(floor) == next_floor:
   
            return;

        if elevator.elevator_env.heuristic == True:

            if self.verticals[int(next_floor)].on == False:
                self.requst_action(next_floor)
            elif self.act_fsm.curr_state != fsm.State.Decelerate:
                self.act_fsm.transition(fsm.Event.DecelerateStart)
            elif self.req_floor[self.move] == next_floor:
                self.act_fsm.transition(fsm.Event.DecelerateStart);
                return;

            if len(self.passengers)>0:
                return;


            find  = False
            if self.move == MoveState.up:
                find = self.req_floor[MoveState.down] == next_floor
            
            else:
                find = self.req_floor[MoveState.up] == next_floor
            


            if find:
                self.act_fsm.transition(fsm.Event.DecelerateStart);
           

          
        else:
            #학습중일경우.. 네트웍 어떻게 할지 정하자..
            sss:int = 0;
        

       # 추후에 네트웍과 어떻게 할지 정해지면 처리해야 할 코드
       #if (floorBtnflag[nextfloor]|| requestFloor == nextfloor)
       #{
       #    if (fsm.GetCurrentState() != State.Decelerate)
       #    {
       #        fsm.transition(Event.DecelerateStart);
       #    }
       #
       #    if (requestFloor == nextfloor && fl.iscallRequest(mvstate))
       #    {
       #        AddReward(0.03f);
       #    }
       #
       #    return;
       #}
       #
       #
       #
       #if (fl.iscallRequest(mvstate))
       #{
       #    RequstAction(nextfloor);
       #}
            


    def update_pos(self):

        if self.cool_time > self._building.play_time:
            return;

        y = self.pos.y+clift.movedir[self.move.value]*self.curr_speed*elevator.elevator_env.fixedTime
        self.pos.y = y

        if self.pos.y >= self._building.floors[elevator.elevator_env.floors-1].pos.y: 
            self.pos.y = self._building.floors[elevator.elevator_env.floors-1].pos.y
            #self.move = MoveState.down
            self.act_fsm.transition(fsm.Event.Arrived);
        
        elif self.pos.y <= self._building.floors[0].pos.y: 
            self.pos.y = self._building.floors[0].pos.y
            #self.move = MoveState.up
            self.act_fsm.transition(fsm.Event.Arrived);
           
        self.curr_floor = (self.pos.y/elevator.elevator_env.height)-1;
        self.chk_floor()

 

    def set_dirction(self,m:MoveState):
        self.move = m;


    def set_floorbutton(self,floor:int,bOn:bool):
         
        self.verticals[floor].on = bOn;

        if self.move == MoveState.stop and bOn:   
            
            if floor > self.curr_floor:
                self.set_dirction(MoveState.up);
            
            elif floor < self.curr_floor: 
                self.set_dirction(MoveState.down);
                      
        
    def update(self):
 
        if self.chk_transtime():
            if self.act_fsm.curr_state in self.elevatorAction.keys():
                self.elevatorAction[self.act_fsm.curr_state]();

        self.update_pos()

       


    def requst_action(self,floor:int):
    
        if elevator.elevator_env.heuristic == True:
            if self.act_fsm.curr_state == fsm.State.NormalMove:
                f = self._building.floors[floor]

                if floor == 0:
                    if f.iscall(MoveState.up):                
                        self.act_fsm.transition(fsm.Event.DecelerateStart);
                        return;
                                 
                elif floor == (elevator.elevator_env.floors-1):
                
                    if f.iscall(MoveState.down):
                        self.act_fsm.transition(fsm.Event.DecelerateStart);
                        return;
                    
                elif f.iscall(self.move):
                    self.act_fsm.transition(fsm.Event.DecelerateStart);
                    return;
                    
         
                if len(self.passengers)==0 and self._building.iscall() == False:
                
                    self.act_fsm.transition(fsm.Event.DecelerateStart);
                    return;
              
            return;
        
        if self.act_fsm.curr_state == fsm.State.Ready:    
            if self._building.play_time - reqTime < 0.5:
                return;
        else:
        
            if self.reqState == self.act_fsm.curr_state and self.reqfloor == floor:
                return;

       
        self.RequestDecision();

        self.reqState = fsm.GetCurrentState();
        self.reqdecision_floor = floor;
        self.reqTime = self._building.play_time;

        self.requestDecision = True;

  
    def act_ready(self):
          #아무것도 안하고 대기..
        self.move = MoveState.stop
        self.curr_speed = 0;
        self.set_transition_delay(fsm.Event.End,0.5,False);
        self.requst_action(self.curr_floor);
        #RequstAction((int)GetFloor());

    def act_accelate(self):
        #정상속도로 되기 위해서 가속상태..

        if self.curr_floor ==0 and self.move != MoveState.up:
            self.move = MoveState.up
        elif  self.curr_floor == elevator.elevator_env.floors -1 and self.move != MoveState.down:
            self.move = MoveState.down

        self.curr_speed +=  FIXED_TIME*1;
        if self.curr_speed < self._building._env.speed:
            return;

        self.curr_speed = self._building._env.speed;
        self.act_fsm.transition(fsm.Event.AccelateEnd);

    def act_normalmove(self):
        self.chk_floor();
    

    def act_decelerate(self):    
        nextfloor:int;

        if self.move.up == MoveState.up:
            nextfloor = (int)(self.curr_floor) + 1;

            if nextfloor >= elevator.elevator_env.floors:
                nextfloor = elevator.elevator_env.floors - 1;
        
        else:
            nextfloor = self.curr_floor;
             
        dist = self._building.floors[nextfloor].pos.y - self.pos.y;

        if abs(dist)<= self.curr_speed*FIXED_TIME or abs(dist)<0.09 :
            self.pos.y =  self._building.floors[nextfloor].pos.y;
            self.act_fsm.transition(fsm.Event.Arrived);    
            self.curr_speed = 0;
            return;
        
        if self.curr_speed  < 0.65:
            return;

        self.curr_speed -= FIXED_TIME * elevator.elevator_env.decelerate;


    def act_movestop(self):
        floor = int(self.curr_floor);
        f = self._building.floors[floor]; 

        self.curr_speed =0

        if self.verticals[floor].on or f.iscall(self.move):
            self.act_fsm.transition(fsm.Event.DoorOpenRequest);
        
        elif len(self.passengers)==0 and len(f.passenger_list)>0 :

            if self.move == MoveState.down: 
                self.move = MoveState.up;
            else: 
                self.move =MoveState.down;
            
            self.act_fsm.transition(fsm.Event.DoorOpenRequest);     
        
        elif len(self.passengers) == 0 and len(f.passenger_list)==0:
            self.act_fsm.transition(fsm.Event.EmptyPassenger);
        
        else:
            self.act_fsm.transition(fsm.Event.DoorCloseEnd);
        
        if self.req_floor[self.move] == floor:
            self.req_floor[self.move] = -1

        self.set_floorbutton(floor, False);

        if self.reqdecision_floor == floor:
            self.reqdecision_floor = -1;


    def act_dooropening(self):
        self.set_transition_delay(fsm.Event.DoorOpenEnd,elevator.elevator_env.open);



    def act_dooropened(self):
        boardingDelay =0;
        idx = 0;

        stayfloor:int = self.curr_floor;

        if self.move == MoveState.up:
             stayfloor =  round(self.curr_floor);#Mathf.RoundToInt(currentFloor);
        elif  self.move == MoveState.stop:
            stayfloor = self.curr_floor;
        else:
            stayfloor =  round(self.curr_floor);


        while idx < len(self.passengers):
        
            p:passenger = self.passengers[idx];
            if p.dest_floor == stayfloor:
                del self.passengers[idx];
                boardingDelay += random.uniform(0.6, 1.0);

                refTime = abs((p.start_floor - p.dest_floor) * (elevator.elevator_env.height) / elevator.elevator_env.speed/2);
                #AddReward(refTime / (Time.fixedTime - p.timeWaiting));
                #AddReward(0.0001);

                #self._building.AddDestPassenger();
            
            else:    
                idx+=1

        self.set_transition_delay(fsm.Event.DoorCloseStart, boardingDelay);

        self.take_lift(stayfloor)

    def take_lift(self,floor:int):
         f = self._building.floors[floor];

         #승객태우자.
         #for p in f.passenger_list:
         #    self.t

         idx = 0
         while idx <len(f.passenger_list):
             if self.take_passenger(f.passenger_list[idx]) == True:
                del f.passenger_list[idx]
             else:
                idx+=1
             



    def act_doorclosing(self):
    
        if len(self.passengers) > 0:
            #승객이 있을 경우는 다시 이동을 하도록 셋팅해준다..
            self.set_transition_delay(fsm.Event.DoorCloseEnd, 1);
        else:
            #승객이 없을 경우는 일단 해당층에 서 대기한다.
            self.set_transition_delay(fsm.Event.EmptyPassenger, 1);                                                                            
        
        #textDoor.gameObject.SetActive(True);
        #textPassenger.text = listPassenger.Count.ToString();

    def act_turn(self):
        iii:int = 0;
    
           
    def get_nextfloor(self):
        floor = -1;
        nextfloor = -1;

        self.curr_floor = (self.pos.y/elevator.elevator_env.height) -1;


        if self.move == MoveState.up:
            floor =  self.curr_floor;
            nextfloor = round(self.curr_floor)
        elif self.move == MoveState.stop:
            floor = self.curr_floor;
            nextfloor = floor
        elif self.move == MoveState.down:
            floor = math.ceil(self.curr_floor-0.1);
            nextfloor = round(self.curr_floor)


        return int(floor),int(nextfloor)


    def is_takeable(self):
    
        if self.act_fsm.curr_state != fsm.State.DoorOpened or len(self.passengers) >= elevator.elevator_env.capacity:
            return False;

        return True;

    def take_passenger(self,p):

        if self.is_takeable() == False:
            return False;

        #if(self.curr_floor == req_floor):
        #    AddReward(0.01f);

        self.verticals[p.dest_floor].on = True;

        if len(self.passengers) ==0:
            self.passengers.append(p);
            self.set_transition_delay(fsm.Event.DoorCloseStart, random.uniform(0.6, 1.0),True);    
            return True

        if self.move == MoveState.up and p.dest_floor>self.curr_floor:
            self.passengers.append(p);

            self.set_transition_delay(fsm.Event.DoorCloseStart, random.uniform(0.6, 1.0),True);         
            #SetFloorButton(p.destFloor, True);

         
        elif self.move  == MoveState.down and p.dest_floor < self.curr_floor:  
            self.passengers.append(p);
            self.set_transition_delay(fsm.Event.DoorCloseStart, random.uniform(0.6, 1.0),True);        
            #SetFloorButton(p.destFloor, True);

            #AddReward(1f / (Time.fixedTime - p.timeWaiting));
            #p.timeWaiting = Time.fixedTime;
        
        else:  
            return False;
       
        return True;

    def get_floordist(self,floor:int ,dir:MoveState):
    
        dist:float = floor - self.curr_floor;

        if self.move == MoveState.stop:
            return abs(dist);

        if self.move == MoveState.up:
        
            if dist > 0:
                return abs(dist);
            else:    
                dist += elevator.elevator_env.floors-floor
                return dist;
            
        if  dist <= 0:
            return  dist;
      
        else:                                  
            dist += floor;

        return dist;


    def set_callrequest(self,floor:int,dir:MoveState):

        d:int = dir;
        self.req_floor[d] = floor;

        if self.act_fsm.curr_state == fsm.State.Ready:
            if floor == self.curr_floor:
                self.act_fsm.transition(fsm.Event.DoorOpenRequest);

            if floor > self.curr_floor:
                self.move = MoveState.up
            else:
                self.move = MoveState.down

            self.act_fsm.transition(fsm.Event.Call);
            return True;

        return False;


    def render(self):


        for v in self.verticals:
            v.render()

        x,y = ToScreenPos(self.pos);
        
        pg.draw.rect(SCREEN,GREEN,[x-METERPERPIXEL*1.5,y-METERPERPIXEL*1.5,METERPERPIXEL*3,METERPERPIXEL*3],1)
        DrawText(x,y,str(len(self.passengers)),BLUE)

        if self.move == MoveState.up:
            DrawText(x,y-METERPERPIXEL*1.3,'^',BLUE)
        elif self.move == MoveState.down:
            DrawText(x,y-METERPERPIXEL*1.3,'V',RED)

    



    


class passenger_spawn(object):

  def __init__(self):
    self.step =0;
    self.floor =-1;
    destfloor = [];



class cbuilding(object):



    def __init__(self,env_:object,screen):
        self._env = env_
        
        global FIXED_TIME
        FIXED_TIME = elevator.elevator_env.fixedTime

        global SCREEN
        SCREEN = screen;

        global FONT
        FONT = pg.font.Font('freesansbold.ttf', 16)

        self.lifts :list=[]
        self.floors: list=[]
        
        self.total_passenger:int = 0
        self.rest_passenger:int = 0
        
        self.curr_passenger: int =0
        self.dest_passenger: int =0
        self.add_passenger : int =0
        
        self.simulattion_time: float = 0;
        
        self.start_time :float= 0;
        self.success_count :int = 0;
        self.fail_count :int = 0;
        self._env:object;
        
        self.step:int = 0
        
        self.call_reqreserve:list = [[]];
        self.play_time:float = 0
        self.episiode = 0;
       
      
     
        for i in range(0,env_.elevator_count):
            c = clift(i,self)
            self.lifts.append(c)

        self.init_env()

    def init_env(self):    
        self.rest_passenger = self._env.passenger;
        self.dest_passenger = 0;
        self.simulattion_time = 0;
        self.add_passenger = 0;
        self.play_time = 0;
        self.episiode = self.episiode+1

        global SCREEN_HEIGHT
        global SCREEN_WIDTH
        global METERPERPIXEL
       
        global LIFT_WIDTH

        SCREEN_WIDTH,SCREEN_HEIGHT  = SCREEN.get_size();
        METERPERPIXEL = SCREEN_HEIGHT/(elevator.elevator_env.floors+1)/elevator.elevator_env.height  #미터기준을 정한다..
        LIFT_WIDTH = (SCREEN_WIDTH/METERPERPIXEL/(elevator.elevator_env.elevator_count+1))     


        dist = SCREEN_WIDTH/(elevator.elevator_env.elevator_count+1);
        rest = elevator.elevator_env.elevator_count% 2;
        mok  = elevator.elevator_env.elevator_count/ 2;

     
        for i in range(self._env.elevator_count):
            if i >= len(self.lifts):
                car = clift(self)
                self.lifts.append(car)
    
            self.lifts[i].pos.x =  dist * (i+1);
            self.lifts[i].Init(i,random.randint(0,3))
            #self.lifts[i].Init(i,random.randint(0,elevator.elevator_env.floors-1))


        for i in range(self._env.floors):
            if i >= len(self.floors):
                f = cfloor(self,i)
                self.floors.append(f)
            else:
                self.floors[i].init(i)

            if len(self.call_reqreserve) <= i:
                self.call_reqreserve.append([-1,-1]);
            else:
                self.call_reqreserve[i] = [-1,-1];


        #self.startTime = fixedTime;
        self.step = 0
        self.isDone = False


    def passenger_random(self):

        if self.curr_passenger > self._env.passenger * 0.3:
            return;

        newPassenger = random.randint(0, self.rest_passenger);
        floorPassenger = [0 for i in range(len(self.floors))]
        floorPassenger[0] = random.randint(0, (int)(newPassenger * 0.8));

        rest = newPassenger - floorPassenger[0];


        while rest > 0:  
            floor = random.randint(0, len(self.floors)-1);
            passenger = random.randint(1, rest);
            rest -= passenger;
            floorPassenger[floor] = passenger;
        
         

        for i in range(len(self.floors)):
        
            if floorPassenger[i] > 0:
            
                destlist = self.floors[i].add_passenger(floorPassenger[i]);
                self.add_passenger += floorPassenger[i];
                self.rest_passenger -= floorPassenger[i];
         
        self.simulattion_time+=5;

       
    def simulation_passenger(self):
        self.passenger_random();


    def take_passenger(self):
        for floor in self.floors:
            for lift in self.lifts:            
                floor.take_passenger(lift);


    def add_destpassenger(self):
        self.dest_passenger+=1;
            
        
    def update_lift(self):

        for f in self.floors:
            f.update()

        for lift in self.lifts:
            lift.update();


    def call_request(self,floor:int,dir:MoveState):
      
        if elevator.elevator_env.heuristic == True:
            self.search_nearst_car(floor, dir);
       
           
    def search_nearst_car(self,floor:int,dir:MoveState):
    
        min = 1000000.0;
        dist = 0;
        buttonDir = 0;

        if dir != MoveState.down:
            buttonDir = 1;
        
      
        for lift in self.lifts:
        
            dist = lift.get_floordist(floor,dir)

            if dist < min:
                self.call_reqreserve[floor][buttonDir] = lift.car_no;
                min = dist;
            
     
        if self.call_reqreserve[floor][buttonDir] != -1:
            lift = self.lifts[self.call_reqreserve[floor][buttonDir]];
            lift.set_callrequest(floor, dir);
            return lift.car_no;
        
        return -1;


    def iscall(self):
        for f in self.floors:
            if f.iscall(MoveState.up) or f.iscall(MoveState.down):
                return True;

        return False;


    def update_step(self):

        self.play_time +=0.1;
        if self.step >= elevator.elevator_env.maxstep:
            self.isDone = True;
            return;
       
        self.simulation_passenger();
        self.update_lift();
        self.step+=1




    def render(self):
        SCREEN.fill(BLACK)

        for floor in self.floors:
            floor.render()

        for lift in self.lifts:
            lift.render()

        text = "Deep Lift Ep:"+str(self.episiode)+ " Success:"+str(self.success_count)+" Step:"+str(self.step) 
        DrawText(0,0,text,GREEN)
      


class passenger(object):

    def __init__(self,start:int,dest:int):
        self.start_floor = start;
        self.dest_floor = dest;
        wait_time = 0;


class cfloor(object):
 


   
    def __init__(self,b:cbuilding):
        self.floor_no = 0
        self._building =b
        self.passenger_list= []

    def __init__(self,b:cbuilding,no:int):
        self.floor_no = no
        self._building =b
        self.passenger_list= []
        self.init(no)
       
      
    
    def init(self,f:int):
        self.up_call: bool = False
        self.down_call: bool = False
        self.pos = Vector3(0,0,0);
        self.pos.y = self._building._env.height*(f+1)
        #self.po. = ToGamePos(0,self._building._env.height*(f+1)*METERPERPIXEL)
        self.passenger_list.clear()
        self.checkTime = self._building.play_time


    def iscall(self,dir:MoveState):
        if dir == MoveState.up:
            return self.up_call;
        elif dir == MoveState.down:
            return self.down_call;
        else :
            return False;

   #def iscall(self):
   #    if self.down_call== True or self.up_call == True :
   #        return True;
   #
   #    return False;

    def setpassenger(self,count:int):
        self.passenger = count;

        for i in range(0,count):
            p: passenger = passenger(floor_no,floor_no);

            while True:
                p.dest_floor = random.randint(0,10);
                if  p.dest_floor != p.start_floor:
                    break;      
                      
            self.passenger_list.append(p);

    def update(self):
        if self._building.play_time - self.checkTime> 1:
            self.chk_callbutton()

    def chk_callbutton(self):

        self.up_call = False;
        self.down_call = False;
        for p  in self.passenger_list:
            if p.dest_floor> self.floor_no:
                self.up_call = True;
            else:
                self.down_call = True;

            if self.down_call == True and self.up_call == True:
                break;

        self.set_button(MoveState.down,self.down_call);
        self.set_button(MoveState.up,self.up_call);

        self.checkTime = self._building.play_time


    def enter_lift(self,car :clift):
        if car.is_enterable() != True:
            return;

        delay =0;
        idx  =0;
       # while self.passenger_list.count >1:
       #     if car.is_enterable() != True:
       #         break;


    def add_passenger(self,passenger_count:int):

        destList:list = [];

        for  i in range(passenger_count):

            p = passenger(self.floor_no,self.floor_no);
            while True:     
                p.dest_floor = random.randint(0,elevator.elevator_env.floors-1);
                if p.dest_floor != self.floor_no:
                    destList.append(p);
                    break;
                
            self.passenger_list.append(p);


        #textPassenger.text = listPassenger.Count.ToString();
        self.chk_callbutton();
        
        return destList;


    def take_passenger(self,lift:clift):

        delay=0;

        idx = 0;
        while idx<  len(self.passenger_list):
           
            if (lift.take_passenger(self.passenger_list[idx])):
                self.passenger_list.remove(idx);
            else:
                ++idx;


    def set_button(self,dir:MoveState,on:bool):
         #textCallButton[(int)dir].gameObject.SetActive(bOn);

        if on == True:   
            self._building.call_request(self.floor_no, dir);


    def render(self):
        x,y = ToScreenPos(self.pos);
        pg.draw.line(SCREEN,WHITE,[x,y],[SCREEN_WIDTH,y],3)

        DrawText(x+METERPERPIXEL,y-METERPERPIXEL,str(self.floor_no),BLUE)
        DrawText(x+METERPERPIXEL,y+3,str(len(self.passenger_list)),RED)

        if self.up_call:
            DrawText(x+METERPERPIXEL*3,y-METERPERPIXEL,'^',BLUE)
        
        if self.down_call:
            DrawText(x+METERPERPIXEL*3,y+3,'V',RED)



class verticalLine(object):

    floor:int
    pos:Vector3
    on:bool
    def __init__(self,floor:int,x:int):
        self.floor = floor
        self.pos = Vector3(x,(floor+1)*elevator.elevator_env.height,0)
        self.on = False;

    def Init(self):
        self.on = False;


    def Set(on:bool):
        self.on = on;


    def render(self):
        x,y = ToScreenPos(self.pos)
        pg.draw.line(SCREEN,GRAY,[x,y],[x,y+elevator.elevator_env.height*METERPERPIXEL],3)








        

            
        





