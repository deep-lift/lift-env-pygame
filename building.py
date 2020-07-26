import random
import math
import pygame as pg
import numpy as np
from elevator import *
from fsm import *
from render import *
from argslist import *
from observation import *


class Lift(object):

    move_dir = [0, -1, 1]  # todo : 정지 아래 위 방향 이리 되는지 확인

    def __init__(self, scr, no: int, bd: object):
        self.scr = scr
        self.car_no = no
        self._building = bd
        self.max_speed = bd._env.speed
        self.curr_speed = 0
        self.height = bd._env.height
        self.act_fsm = FSM(State.Ready)
        self.pos = Vector3(0, 0, 0)
        self.passengers = []
        self.move = MoveState.STOP
        self.curr_floor = 0

        self.act_fsm.add_transition(State.Ready, Event.Call, State.Accelate)
        self.act_fsm.add_transition(State.Ready, Event.DoorOpenRequest, State.DoorOpening)

        self.act_fsm.add_transition(State.Accelate, Event.AccelateEnd, State.NormalMove)
        self.act_fsm.add_transition(State.Accelate, Event.DecelerateStart, State.Decelerate)
        self.act_fsm.add_transition(State.Accelate, Event.Arrived, State.MoveStop)

        self.act_fsm.add_transition(State.NormalMove, Event.DecelerateStart, State.Decelerate)
        #self.act_fsm.add_transition(State.NormalMove, Event.Arrived, State.MoveStop);

        self.act_fsm.add_transition(State.Decelerate, Event.Arrived, State.MoveStop)

        self.act_fsm.add_transition(State.MoveStop, Event.DoorOpenRequest, State.DoorOpening)
        self.act_fsm.add_transition(State.MoveStop, Event.EmptyPassenger, State.Ready)
        self.act_fsm.add_transition(State.MoveStop, Event.DoorCloseEnd, State.Accelate)

        self.act_fsm.add_transition(State.DoorOpening, Event.DoorOpenEnd, State.DoorOpened)
        self.act_fsm.add_transition(State.DoorOpened, Event.DoorCloseStart, State.DoorClosing)
        self.act_fsm.add_transition(State.DoorClosing, Event.DoorCloseEnd, State.Accelate)
        self.act_fsm.add_transition(State.DoorClosing, Event.EmptyPassenger, State.Ready)

        self.elevatorAction = dict()

        self.elevatorAction[State.Ready] = self.act_ready                # 문닫고 멈춰있는 상태
        self.elevatorAction[State.NormalMove] = self.act_normalmove      # 위아래 어느쪽이든 정상적인 이동상태
        self.elevatorAction[State.Decelerate] = self.act_decelerate      # 다음층에 멈추기 위한 감속상태
        self.elevatorAction[State.MoveStop] = self.act_movestop          # 멈춘상태
        self.elevatorAction[State.DoorOpening] = self.act_dooropening    # 문열는중
        self.elevatorAction[State.DoorOpened] = self.act_dooropened      # 문열린 상태에서 승객내리고 타고
        self.elevatorAction[State.DoorClosing] = self.act_doorclosing    # 문닫히는 동안.
        self.elevatorAction[State.Accelate] = self.act_accelate          # 이동에 대한 가속상태
        self.elevatorAction[State.Turn] = self.act_turn

        self.state = observation()
        self.reward = 0
        

        self.verticals = []

    def reset(self, start_floor: int):
        self.curr_floor = start_floor
        self.pos.x = LIFT_WIDTH*(self.car_no+1)
        self.pos.y = self.height * (start_floor + 1)
        self.move = MoveState.STOP
        self.passengers.clear()
        self.req_state = State.End
        self.req_floor = -1
        self.req_time  = 0
        self.reward = 0
        self.req_decision = True
        self.cool_time = 0
        self.req_decision_floor = -1
        self.next_transition_time = 0
        self.next_event = State.End
        self.req_floor = {MoveState.STOP:-1, MoveState.DOWN:-1, MoveState.UP:-1}
        self.action = MoveState.STOP
        self.done = False
        self.state.reset()
        self.act_fsm.curr_state = State.Ready
        self.recved_decision_floor =-1;  #네트웍에서 액션으로 받은 목적지 층

        for i in range(self._building._env.floors):
            if i >= len(self.verticals):
                v = VerticalLine(self.scr, self._building._env, i, self.pos.x)
                self.verticals.append(v)
            else:
                self.verticals[i].reset()

    def make_state(self, state: observation):

        ######################################################################################
        # Global Observation
        state.add(self._building.rest_passenger)  # 남은 승객수

        for f in self._building.floors:
            state.add(len(f.passengers))            # 각층 대기 승객 수
            state.add(int(f.is_call(MoveState.DOWN)))    # 각층 DOWN 버튼 눌린 여부
            state.add(int(f.is_call(MoveState.UP)))      # 각층 UP 버튼 눌린 여부

        
        ######################################################################################

        ######################################################################################
        # Local Observation
        state.add(self.curr_floor)                  # 해당 리프트 현재 층수(int->float) 
        state.add(Lift.move_dir[self.move.value] * self.curr_speed)  #엘베속도..
        state.add(int(self.act_fsm.curr_state.value))          # 해당 리프트의 FSM 상태
        state.add(len(self.passengers))             # 해당 리프트의 탑승 승객 수

        for v in self.verticals:
            state.add(int(v.on))                         # 엘레베이터 버튼 on 여부
        ######################################################################################

    def collect_obs(self):
        self.state.reset()
        for lift in self._building.lifts:
            lift.make_state(self.state)
        return self.state.obsvector

    def decision_floor(self, decision_floor):

        if not self.req_decision:
            return

        self.reward = 0
        self.req_decision = False

        curfloor, next_floor = self.get_nextfloor()
       
        if decision_floor == self.curr_floor:
            if self.act_fsm.curr_state == State.Ready:
                self.act_fsm.transition(Event.DoorOpenRequest)
            else:
                self.act_fsm.transition(Event.Arrived)
            return

        self.recved_decision_floor = decision_floor
        if decision_floor > self.curr_floor:
            self.set_direction(MoveState.UP)
        else:
            self.set_direction(MoveState.DOWN)

        self.act_fsm.transition(Event.Call)
    

        
    def decision_action(self, action):
        if not self.req_decision:
            return

        self.reward = 0
        self.req_decision = False
        
        floor, next_floor = self.get_nextfloor()

        f = self._building.floors[next_floor]
        
        action_state = MoveState(action)
        if action_state == MoveState.STOP:
            if self.act_fsm.curr_state == State.Ready:
                while len(f.passengers)>0:
                    dir = random.randrange(1, 3)

                    if f.is_call(MoveState(dir)):
                        self.act_fsm.transition(Event.DoorOpenRequest)
                        self.set_direction(dir)
                        return

            self.act_fsm.transition(Event.DecelerateStart)
            return

        elif action_state == MoveState.DOWN:  # 현재 이동중 방향과 다르게 왔을 경우..
            if floor == 0:
                return

            if self.move != action:

                if floor != 0 and len(self.passengers)>0 and self.move != MoveState.STOP:
                    return

                if len(self.passengers)>0:
                    return

                elif self.curr_speed >0:
                    return

            self.set_direction(action_state)
            self.act_fsm.transition(Event.Call)
        else:

            if floor == self._building._env.floors -1:
                return

            if self.move != action:

                
                if floor != self._building._env.floors-1 and len(self.passengers)>0 and self.move != MoveState.STOP:
                    return

                if len(self.passengers)>0:
                    return
                elif self.curr_speed >0:
                    #self.act_fsm.transition(Event.DecelerateStart)
                    return

            self.set_direction(action_state)
            self.act_fsm.transition(Event.Call)

    def is_enterable(self):
        return True

    def set_transition_delay(self, event: Event, delay: float, add: bool = False):
        if add:
            self.next_transition_time += delay
        else:
            self.next_transition_time = delay
        self.next_event = event

    def chk_transtime(self):
        if self.next_event == Event.End and self.next_transition_time == 0:
            return True

        self.next_transition_time -= self._building._env.fixedTime

        if self.next_transition_time > 0:
            return False

        self.act_fsm.transition(self.next_event)
        self.next_event = Event.End
        self.next_transition_time = 0

        return True

    def chk_floor(self):
        floor = -1
        next_floor = -1
        floor, next_floor = self.get_nextfloor()


       # if int(floor) != next_floor and self.verticals[next_floor].on:
       #     self.act_fsm.transition(Event.DecelerateStart)
       #     return
               
 

        if self._building._env.heuristic:

            if int(floor) == next_floor:
                return

            if not self.verticals[int(next_floor)].on and not self._building.floors[next_floor].is_call(self.move):
                self.request_action(next_floor)
            elif self.act_fsm.curr_state != State.Decelerate:
                self.act_fsm.transition(Event.DecelerateStart)
            elif self.req_floor[self.move] == next_floor:
                self.act_fsm.transition(Event.DecelerateStart)
            elif next_floor == 0 or next_floor  ==  self._building._env.floors -1:
                 self.act_fsm.transition(Event.DecelerateStart)

            if len(self.passengers) > 0:
                return

            find = False

            if self.move == MoveState.UP:
                find = self.req_floor[MoveState.DOWN] == next_floor
            else:
                find = self.req_floor[MoveState.UP] == next_floor

            if find:
                self.act_fsm.transition(Event.DecelerateStart)

            return


        if self.recved_decision_floor == self.curr_floor and self.curr_speed ==0:
            self.recved_decision_floor = -1

        
        if next_floor == self.recved_decision_floor:
            self.act_fsm.transition(Event.DecelerateStart)

       
        self.request_action(next_floor)


    def update_pos(self):
        if self.cool_time > self._building.play_time:
            return

        y = self.pos.y + Lift.move_dir[self.move.value] * self.curr_speed * self._building._env.fixedTime
        self.pos.y = y

        if self.pos.y >= self._building.floors[self._building._env.floors - 1].pos.y:
            self.pos.y = self._building.floors[self._building._env.floors - 1].pos.y
            self.set_direction(MoveState.DOWN);

        elif self.pos.y <= self._building.floors[0].pos.y:
            self.pos.y = self._building.floors[0].pos.y
            self.set_direction(MoveState.UP);
            
        self.curr_floor = ((self.pos.y / self._building._env.height) - 1)+0.01
        self.chk_floor()

    def set_direction(self, m):
        self.move = MoveState(m)

    def set_floorbutton(self, floor: int, on: bool):
        #self.verticals[floor].on = on

        if self.move == MoveState.STOP and on:
            if floor > self.curr_floor:
                self.set_direction(MoveState.UP)
            elif floor < self.curr_floor:
                self.set_direction(MoveState.DOWN)

    def update(self):
        if self.chk_transtime():
            if self.act_fsm.curr_state in self.elevatorAction.keys():
                self.elevatorAction[self.act_fsm.curr_state]()

        self.update_pos()

    def request_action(self, floor: int):

        if self._building._env.heuristic:
            if self.act_fsm.curr_state == State.NormalMove:
                f = self._building.floors[floor]

                if floor == 0:
                    if f.is_call(MoveState.UP):
                        self.act_fsm.transition(Event.DecelerateStart)
                        return
                elif floor == (self._building._env.floors - 1):
                    if f.is_call(MoveState.DOWN):
                        self.act_fsm.transition(Event.DecelerateStart)
                        return
                elif f.is_call(self.move):
                    self.act_fsm.transition(Event.DecelerateStart)
                    return

                if len(self.passengers) == 0 and not self._building.is_call():
                    self.act_fsm.transition(Event.DecelerateStart)
                    return
            return

     
        if self.act_fsm.curr_state == State.Ready:
            if self._building.play_time - self.req_time < 0.5:
                return
       
        elif self.req_decision_floor == floor:
                return
        else:
             self.req_decision_floor = -1

        
        if self._building._env.action_to_floor  == 0:
            self.request_decision(floor)


    def request_decision(self, floor):

        if self.recved_decision_floor >-1:
            return

        self.req_state = self.act_fsm.get_current_state()
        self.req_decision_floor = floor
        self.req_time = self._building.play_time
        self.req_decision = True

    def act_ready(self): #아무것도 안하고 대기..
        self.move = MoveState.STOP
        self.curr_speed = 0
        self.set_transition_delay(Event.End,0.5,False)

        if self._building._env.action_to_floor ==0:
            self.request_action(self.curr_floor)
        else:
            self.request_decision(self.curr_floor)


        

    def act_accelate(self):  # 정상속도로 되기 위해서 가속상태..

        #if self.curr_floor ==0 and self.move != MoveState.UP:
        #    self.move = MoveState.UP
        #elif self.curr_floor == self._building._env.floors -1 and self.move != MoveState.DOWN:
        #    self.move = MoveState.DOWN

        if self._building._env.action_to_floor>0:
            self.request_decision(self.curr_floor)

        self.curr_speed += self._building._env.fixedTime*1
        if self.curr_speed < self._building._env.speed:
            return

        self.curr_speed = self._building._env.speed
        self.act_fsm.transition(Event.AccelateEnd)

    def act_normalmove(self):
        self.chk_floor()

    def act_decelerate(self):

        nextfloor:int = 0
        if self.move == MoveState.UP:
            nextfloor = (int)(self.curr_floor) + 1

            if nextfloor >= self._building._env.floors:
                nextfloor = self._building._env.floors - 1
        else:
            nextfloor = self.curr_floor

        nextfloor = int(nextfloor)

        dist = self._building.floors[nextfloor].pos.y - self.pos.y

        if dist*self.move_dir[self.move.value]<=0 or abs(dist) <= 0.65:
            self.pos.y =  self._building.floors[nextfloor].pos.y
            self.act_fsm.transition(Event.Arrived)
            self.curr_speed = 0
            return

        if self.curr_speed < 0.65:
            return

        self.curr_speed -= FIXED_TIME * self._building._env.decelerate

    def act_movestop(self):
        floor = int(self.curr_floor)
        f = self._building.floors[floor]
        self.curr_speed = 0

        if self.verticals[floor].on or f.is_call(self.move):
            self.act_fsm.transition(Event.DoorOpenRequest)

        elif len(self.passengers)==0 and len(f.passengers)>0 :

            if self.move == MoveState.DOWN:
                self.move = MoveState.UP
            else:
                self.move = MoveState.DOWN

            self.act_fsm.transition(Event.DoorOpenRequest)

        elif len(self.passengers) == 0 and len(f.passengers)==0:
            self.act_fsm.transition(Event.EmptyPassenger)

        else:
            self.act_fsm.transition(Event.DoorCloseEnd)

        if self.req_floor[self.move] == floor:
            self.req_floor[self.move] = -1

        self.set_floorbutton(floor, False)

    def act_dooropening(self):
        self.set_transition_delay(Event.DoorOpenEnd, self._building._env.open)

    def act_dooropened(self):
        boardingDelay =0
        idx = 0
        stayfloor:int = self.curr_floor

        if self.move == MoveState.UP:
            stayfloor = round(self.curr_floor)#Mathf.RoundToInt(currentFloor)
        elif  self.move == MoveState.STOP:
            stayfloor = round(self.curr_floor)
        else:
            stayfloor = round(self.curr_floor)

        while idx < len(self.passengers):
            p: Passenger = self.passengers[idx]
            if p.dest_floor == stayfloor:
                del self.passengers[idx]
                self.verticals[int(stayfloor)].set(False)
                boardingDelay += random.uniform(0.6, 1.0)
                refTime = abs((p.start_floor - p.dest_floor) * self._building._env.height / self._building._env.speed / 2)

                self.reward = self.reward + 1
                self._building.dest_passenger+=1;

            else:
                idx += 1

        self.set_transition_delay(Event.DoorCloseStart, boardingDelay)
        self.take_lift(stayfloor)

    def take_lift(self, floor:int):
        f = self._building.floors[floor]
        idx = 0
        while idx <len(f.passengers):
            if self.take_passenger(f.passengers[idx]) == True:
                del f.passengers[idx]
            else:
                idx+=1

    def act_doorclosing(self):
        if len(self.passengers) > 0:
            self.set_transition_delay(Event.DoorCloseEnd, 1)  # 승객이 있을 경우는 다시 이동을 하도록 셋팅
        else:
            self.set_transition_delay(Event.EmptyPassenger, 1)  # 승객이 없을 경우는 일단 해당층에 서 대기



    def act_turn(self):
        raise NotImplementedError

    def get_nextfloor(self):
        floor = -1
        nextfloor = -1

        self.curr_floor = (self.pos.y / self._building._env.height) - 1

        if self.move == MoveState.UP:
            floor = self.curr_floor
            nextfloor = round(self.curr_floor)
        elif self.move == MoveState.STOP:
            floor = self.curr_floor
            nextfloor = floor
        elif self.move == MoveState.DOWN:
            floor = math.ceil(self.curr_floor)
            nextfloor = round(self.curr_floor)

        return int(floor), int(nextfloor)

    def is_takeable(self):
        if self.act_fsm.curr_state != State.DoorOpened or len(self.passengers) >= self._building._env.capacity:
            return False

        return True

    def take_passenger(self,p):
        if self.is_takeable() == False:
            return False

        if len(self.passengers) ==0:
            self.passengers.append(p)
            self.set_transition_delay(Event.DoorCloseStart, random.uniform(0.6, 1.0),True)
            self.verticals[p.dest_floor].set(True)
            return True

        if self.move == MoveState.UP and p.dest_floor>self.curr_floor:
            self.passengers.append(p)
            self.set_transition_delay(Event.DoorCloseStart, random.uniform(0.6, 1.0),True)
            self.verticals[p.dest_floor].set(True)
            #SetFloorButton(p.destFloor, True)
        elif self.move  == MoveState.DOWN and p.dest_floor < self.curr_floor:
            self.passengers.append(p)
            self.set_transition_delay(Event.DoorCloseStart, random.uniform(0.6, 1.0),True)
            self.verticals[p.dest_floor].set(True)

            #승객 태울때 리워드 필요..

            #SetFloorButton(p.destFloor, True)
            #AddReward(1f / (Time.fixedTime - p.timeWaiting))
            #p.timeWaiting = Time.fixedTime
        else:
            return False

        return True

    def get_floor_dist(self, floor):
        dist = abs(floor - self.curr_floor)
        if self.move == MoveState.STOP:
            return abs(dist)

        if self.move == MoveState.UP:
            if dist > 0:
                return abs(dist)
            else:
                dist += self._building._env.floors - floor
                return dist

        if dist <= 0:
            return dist
        else:
            dist += floor

        return dist

    def set_callrequest(self, floor:int, direction: MoveState):
        #self.req_floor[direction] = floor

        if self.act_fsm.curr_state == State.Ready:
            if floor == self.curr_floor:
                self.act_fsm.transition(Event.DoorOpenRequest)

            if floor > self.curr_floor:
                self.move = MoveState.UP
            else:
                self.move = MoveState.DOWN

            self.act_fsm.transition(Event.Call)
            return True

        return False

    def render(self):
        for v in self.verticals:
            v.render()
        x, y = ToScreenPos(self.pos)
        pg.draw.rect(self.scr,
                     GREEN,
                     [x - METER_PER_PIXEL * 1.5, y - METER_PER_PIXEL * 1.5, METER_PER_PIXEL * 3, METER_PER_PIXEL * 3],
                     1)
        DrawText(self.scr, x, y, str(len(self.passengers)), BLUE)

        if self.move == MoveState.UP:
            DrawText(self.scr, x, y - METER_PER_PIXEL * 1.3, '^', BLUE)
        elif self.move == MoveState.DOWN:
            DrawText(self.scr, x, y - METER_PER_PIXEL * 1.3, 'V', RED)


class PassengerSpawn(object):
    def __init__(self,step,floor):
        self.step = step
        self.floor = floor
        self.passengers = []



class Building(object):

    def __init__(self, env: object, scr):
        self._env = env
        self.scr = scr

        self.lifts = []
        self.floors = []

        self.total_passenger: int = 0
        self.rest_passenger: int = 0
        self.curr_passenger: int = 0

        self.start_time: float = 0
        self.success_count: int = 0
        self.fail_count: int = 0
        self._env: object
        self.step: int = 0

        self.call_reqreserve = [[]]
        self.play_time: float = 0
        self.episode = 0
        self.is_done = False
        self.rest_passenger = self._env.passenger
        self.dest_passenger = 0
        self.add_passenger = 0
        self.play_time = 0

        self.scenario_recorde = True
        self.passenger_scenario = []

        SCREEN_WIDTH, SCREEN_HEIGHT = self.scr.get_size()
        METER_PER_PIXELPIXEL = SCREEN_HEIGHT / (self._env.floors + 1) / self._env.height
        LIFT_WIDTH = (SCREEN_WIDTH / METER_PER_PIXELPIXEL / (self._env.elevator_count + 1))

        self.reset()

    def reset(self):
        for i in range(self._env.elevator_count):
            if i >= len(self.lifts):
                car = Lift(self.scr, i, self)
                self.lifts.append(car)
            self.lifts[i].reset(random.randint(0, 3))

        for i in range(self._env.floors):
            if i >= len(self.floors):
                f = Floor(self.scr, self, i)
                self.floors.append(f)
            else:
                self.floors[i].reset()

            if len(self.call_reqreserve) <= i:
                self.call_reqreserve.append([-1, -1])
            else:
                self.call_reqreserve[i] = [-1, -1]

        self.step = 0
        self.play_time = 0
        self.is_done = False
        self.dest_passenger = 0
        self.add_passenger = 0
        self.scenario_index = 0
       
        self.rest_passenger = self._env.passenger
        self.episode = self.episode + 1


    def get_state(self):
        obs_list = []

        for lift in self.lifts:
            obs_list.append(lift.collect_obs())

        return obs_list;

    def env_info(self):
        observations = []
        rewards = []
        dones = []
        requested_agents = []

        if self.step >= self._env.max_step and self.dest_passenger - self._env.passenger < 0:
            for lift in self.lifts:
                # print('episode end.. penalty?')
                # todo : 여기에는 리워드가 최종으로 들어가는 부분. 마지막에 +도달한 승객 - 모든 승객 --> 배달이 다 안되었을 경우는 마이너스를 무조건 받는다.
                lift.reward += (self.dest_passenger - self._env.passenger) / N_AGENTS

        for lift in self.lifts:
            #if lift.req_decision or self.is_done:
            observations.append(lift.collect_obs())
            rewards.append(lift.reward)
            requested_agents = lift.req_decision
            dones.append(self.is_done)

       
        return np.asarray(observations), np.asarray(rewards), np.asarray(dones),np.asarray(requested_agents)

    def simulation_passenger(self):

        if self.scenario_recorde == True or self._env.fixed_scenario == False :
            self.passenger_random_spawn()      
        else: 
            self.passenger_scenario_spawn()
       


    def passenger_random_spawn(self):

        if self.curr_passenger > self._env.passenger * 0.3:
            return

        new_passenger = random.randint(0, self.rest_passenger)

        # 난이도 조절 #1 : 1층에서 승객들이 나오게 변경
        floor_passenger = [0 for i in range(len(self.floors))]
        floor_passenger[0] = random.randint(0, (int)(new_passenger * 0.8))
        rest = new_passenger - floor_passenger[0]

        # 여전히 랜덤한 요소가 많음. 반영이 안된건가?
        while rest > 0:
            floor = random.randint(0, len(self.floors)-1)
            passenger = random.randint(1, rest)
            rest -= passenger
            floor_passenger[floor] = passenger

        for i in range(len(self.floors)):
            if floor_passenger[i] > 0:
                dest_list = self.floors[i].add_passenger(floor_passenger[i])
                self.add_passenger += floor_passenger[i]
                self.rest_passenger -= floor_passenger[i]

                if  self.scenario_recorde == True:
                    spawn = PassengerSpawn(self.step,1);     
                    spawn.passengers = dest_list;
                    self.passenger_scenario.append(spawn);



    def passenger_scenario_spawn(self):

         if self.is_done == True:
             return
                
         while  self.scenario_index < len(self.passenger_scenario):
        
            if self.passenger_scenario[self.scenario_index].step <= self.step:
            
                self.floors[self.passenger_scenario[self.scenario_index].floor].add_passenger_list(self.passenger_scenario[self.scenario_index].passengers)
                
                self.add_passenger += len(self.passenger_scenario[self.scenario_index].passengers);
                self.rest_passenger -= len(self.passenger_scenario[self.scenario_index].passengers);
                self.scenario_index+=1;
            
            else: 
                break;
           
        

    def take_passenger(self):
        for floor in self.floors:
            for lift in self.lifts:
                floor.take_passenger(lift)

    def add_destpassenger(self):
        self.dest_passenger += 1

    def update_lift(self):
        for f in self.floors:
            f.update()

        for lift in self.lifts:
            lift.update()

    def call_request(self, floor: int, direction: MoveState):
        if self._env.heuristic:
            self.search_nearest_car(floor, direction)
       
    def search_nearest_car(self, floor:int, direction: MoveState): # script code
        min = 1000000.0
        dist = 0
        button_dir = 0

        if direction != MoveState.DOWN:
            button_dir = 1

        for lift in self.lifts:
            dist = lift.get_floor_dist(floor)

            if dist < min:
                self.call_reqreserve[floor][button_dir] = lift.car_no
                min = dist

        if self.call_reqreserve[floor][button_dir] != -1:
            lift = self.lifts[self.call_reqreserve[floor][button_dir]]
            lift.set_callrequest(floor, direction)
            return lift.car_no

        return -1

    def is_call(self):
        for f in self.floors:
            if f.is_call(MoveState.UP) or f.is_call(MoveState.DOWN):
                return True
        return False

    def update_step(self):

        self.play_time += self._env.fixedTime  # todo : 단위가 궁금!

        self.simulation_passenger()
        self.update_lift()
        self.step += 1

        if self.step >= self._env.max_step or self.is_success():
            self.is_done = True
            self.scenario_recorde = False  # todo : 이것의 용도는?

        if self._env.heuristic:
            return [],[],[],[]

        return self.env_info()

    def is_success(self):
        if self.rest_passenger > 0:
            return False
        for lift in self.lifts:
            if len(lift.passengers) > 0:
                return False
        for floor in self.floors:
            if len(floor.passengers) > 0:
                return False

        for lift in self.lifts:
            lift.reward = lift.reward * 5

        self.success_count+=1

        return True

    def decision_actions(self, actions):
        no = 0
        for action in actions:
            if self._env.action_to_floor <0:
                self.lifts[no].decision_action(action)
            else:
                self.lifts[no].decision_floor(action)

            no = no + 1

    def render(self):
        self.scr.fill(WHITE)
        for floor in self.floors:
            floor.render()
        for lift in self.lifts:
            lift.render()
        text = "Deep Lift Episode :" + str(self.episode) +" Rest:" +str(self._env.passenger-self.dest_passenger)+", Success :" + str(self.success_count) + ", Step :" + str(self.step)
        DrawText(self.scr, 0, 0, text, BLACK)


class Floor:
    def __init__(self, scr, b: Building):
        self.scr = scr
        self.floor_no = 0
        self._building = b
        self.passengers = []

    def __init__(self, scr, b: Building, no):
        self.scr = scr
        self.floor_no = no
        self._building = b
        self.passengers = []
        self.pos = Vector3(0, 0, 0)
        self.pos.y = self._building._env.height * (no + 1)
        self.reset()

    def reset(self):
        self.up_call: bool = False
        self.down_call: bool = False
        self.passengers.clear()
        self.checkTime = self._building.play_time

  
    def is_call(self, direction: MoveState):
        if direction == MoveState.UP:
            return self.up_call
        elif direction == MoveState.DOWN:
            return self.down_call
        else:
            return False

    def update(self):
        # todo : play_time이 마지막 체크시점보다 더 지났을 경우라는건가?
        if self._building.play_time - self.checkTime > 1:
            self.chk_call_button()

    def chk_call_button(self):
        self.up_call = False
        self.down_call = False
        for p in self.passengers:
            if p.dest_floor > self.floor_no:
                self.up_call = True
            else:
                self.down_call = True

        self.set_button(MoveState.DOWN, self.down_call)
        self.set_button(MoveState.UP, self.up_call)
        self.checkTime = self._building.play_time

    def add_passenger(self, passenger_count: int):
        dest_list = []
        for i in range(passenger_count):
            p = Passenger(self.floor_no, self.floor_no)
            while True:
                p.dest_floor = random.randint(0, self._building._env.floors - 1)
                if p.dest_floor != self.floor_no:
                    dest_list.append(p)
                    break
            self.passengers.append(p)
        self.chk_call_button()
        return dest_list

    def add_passenger_list(self,destlist:list):
        for i in range(len(destlist)):
            p = Passenger(self.floor_no, destlist[i].dest_floor)
            self.passengers.append(p)

        self.chk_call_button()

             
          
    def take_passenger(self, lift: Lift):
        idx = 0
        while idx < len(self.passengers):
            if lift.take_passenger(self.passengers[idx]):
                self.passengers.remove(idx)
            else:
                ++idx

    def set_button(self, direction: MoveState, on: bool):
        if on:
            self._building.call_request(self.floor_no, direction)

    def render(self):
        x, y = ToScreenPos(self.pos)
        pg.draw.line(self.scr, BLACK, [x, y], [SCREEN_WIDTH, y], 3)

        DrawText(self.scr, x + METER_PER_PIXEL, y - METER_PER_PIXEL, str(self.floor_no), BLUE)
        DrawText(self.scr, x + METER_PER_PIXEL, y + 3, str(len(self.passengers)), RED)

        if self.up_call:
            DrawText(self.scr, x + METER_PER_PIXEL * 3, y - METER_PER_PIXEL, '^', BLUE)

        if self.down_call:
            DrawText(self.scr, x + METER_PER_PIXEL * 3, y + 3, 'v', RED)


class VerticalLine(object):
    def __init__(self, scr, env, floor: int, x: int):
        self.scr = scr
        self.env = env
        self.floor = floor
        self.pos = Vector3(x, (floor + 1) * self.env.height, 0)
        self.on = False
        self.reset()

    def reset(self):
        self.on = False

    def set(self, on: bool):
        self.on = on

    def render(self):
        x, y = ToScreenPos(self.pos)
        pg.draw.line(self.scr,
                     GRAY,
                     [x, y],
                     [x, y + self.env.height * METER_PER_PIXEL], 3
                     )

        if self.on:
            DrawText(self.scr, x + METER_PER_PIXEL * 3, y + 3, 'o', RED)
