import random
import math
import pygame as pg

from elevator import *
from fsm import *
from render import *
from argslist import *


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

        self.act_fsm.add_transition(State.Ready, Event.Call, State.Accelate)
        self.act_fsm.add_transition(State.Ready, Event.DoorOpenRequest, State.DoorOpening)

        self.act_fsm.add_transition(State.Accelate, Event.AccelateEnd, State.NormalMove)
        self.act_fsm.add_transition(State.Accelate, Event.DecelerateStart, State.NormalMove)
        self.act_fsm.add_transition(State.Accelate, Event.Arrived, State.MoveStop)

        self.act_fsm.add_transition(State.NormalMove, Event.DecelerateStart, State.Decelerate)
        self.act_fsm.add_transition(State.NormalMove, Event.Arrived, State.MoveStop);


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

    def init(self, no: int, start_floor: int):
        self.curr_floor = start_floor
        self.pos.x = LIFT_WIDTH*(no+1)
        self.pos.y = self.height * (start_floor + 1)
        self.move = MoveState.STOP
        self.passengers.clear()
        self.reqState = State.End
        self.reqfloor = -1
        self.reqTime  = 0
        self.requestDecision = False
        self.cool_time = 0
        self.reqdecision_floor = -1
        self.next_transition_time =0
        self.next_event = State.End
        self.req_floor = {MoveState.STOP:-1, MoveState.DOWN:-1, MoveState.UP:-1}
        self.verticals = []

        for i in range(self._building._env.floors):
            vline = VerticalLine(self.scr, self._building._env, i, self.pos.x)
            self.verticals.append(vline)

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

        if int(floor) == next_floor:
            return

        if self._building._env.heuristic:
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
        else:
            raise NotImplementedError

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
        self.move = m

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
            if self._building.play_time - self.reqTime < 0.5:
                return
        else:

            if self.reqState == self.act_fsm.curr_state and self.reqfloor == floor:
                return

        self.RequestDecision()

        # todo : 여기 맞는건가?!!
        self.reqState = self.act_fsm.get_current_state()
        self.reqdecision_floor = floor
        self.reqTime = self._building.play_time
        self.requestDecision = True


    def act_ready(self): #아무것도 안하고 대기..
        self.move = MoveState.STOP
        self.curr_speed = 0
        self.set_transition_delay(Event.End,0.5,False)
        self.request_action(self.curr_floor)
        #RequstAction((int)GetFloor())

    def act_accelate(self):  # 정상속도로 되기 위해서 가속상태..


        #if self.curr_floor ==0 and self.move != MoveState.UP:
        #    self.move = MoveState.UP
        #elif self.curr_floor == self._building._env.floors -1 and self.move != MoveState.DOWN:
        #    self.move = MoveState.DOWN

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

        elif len(self.passengers)==0 and len(f.passenger_list)>0 :

            if self.move == MoveState.DOWN:
                self.move = MoveState.UP
            else:
                self.move = MoveState.DOWN

            self.act_fsm.transition(Event.DoorOpenRequest)

        elif len(self.passengers) == 0 and len(f.passenger_list)==0:
            self.act_fsm.transition(Event.EmptyPassenger)

        else:
            self.act_fsm.transition(Event.DoorCloseEnd)

        if self.req_floor[self.move] == floor:
            self.req_floor[self.move] = -1

        self.set_floorbutton(floor, False)

        if self.reqdecision_floor == floor:
            self.reqdecision_floor = -1

    def act_dooropening(self):
        self.set_transition_delay(Event.DoorOpenEnd, self._building._env.open)

    def act_dooropened(self):
        boardingDelay =0
        idx = 0
        stayfloor:int = self.curr_floor

        

        if self.move == MoveState.UP:
            stayfloor =  round(self.curr_floor)#Mathf.RoundToInt(currentFloor)
        elif  self.move == MoveState.STOP:
            stayfloor = self.curr_floor
        else:
            stayfloor =  round(self.curr_floor)

        while idx < len(self.passengers):
            p: Passenger = self.passengers[idx]
            if p.dest_floor == stayfloor:
                del self.passengers[idx]
                self.verticals[int(stayfloor)].on = False
                boardingDelay += random.uniform(0.6, 1.0)

                refTime = abs(
                    (p.start_floor - p.dest_floor) * self._building._env.height / self._building._env.speed / 2
                )
            else:
                idx += 1

        self.set_transition_delay(Event.DoorCloseStart, boardingDelay)
        self.take_lift(stayfloor)

    def take_lift(self,floor:int):
        f = self._building.floors[floor]
        idx = 0
        while idx <len(f.passenger_list):
            if self.take_passenger(f.passenger_list[idx]) == True:
                del f.passenger_list[idx]
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

        return int(floor),int(nextfloor)

    def is_takeable(self):
        if self.act_fsm.curr_state != State.DoorOpened or len(self.passengers) >= self._building._env.capacity:
            return False

        return True

    def take_passenger(self,p):
        if self.is_takeable() == False:
            return False

        self.verticals[p.dest_floor].on = True

        if len(self.passengers) ==0:
            self.passengers.append(p)
            self.set_transition_delay(Event.DoorCloseStart, random.uniform(0.6, 1.0),True)
            return True

        if self.move == MoveState.UP and p.dest_floor>self.curr_floor:
            self.passengers.append(p)
            self.set_transition_delay(Event.DoorCloseStart, random.uniform(0.6, 1.0),True)
            #SetFloorButton(p.destFloor, True)
        elif self.move  == MoveState.DOWN and p.dest_floor < self.curr_floor:
            self.passengers.append(p)
            self.set_transition_delay(Event.DoorCloseStart, random.uniform(0.6, 1.0),True)
            #SetFloorButton(p.destFloor, True)
            #AddReward(1f / (Time.fixedTime - p.timeWaiting))
            #p.timeWaiting = Time.fixedTime
        else:
            self.verticals[p.dest_floor].on = False
            return False
        return True

    def get_floor_dist(self, floor):
        dist =  abs(floor - self.curr_floor)
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


class Building(object):

    def __init__(self, env_: object, scr):
        self._env = env_
        self.scr = scr

        self.lifts = []
        self.floors = []

        self.total_passenger: int = 0
        self.rest_passenger: int = 0

        self.curr_passenger: int = 0
        self.dest_passenger: int = 0
        self.add_passenger: int = 0

        self.simulation_time: float = 0
        self.start_time: float = 0
        self.success_count: int = 0
        self.fail_count: int = 0
        self._env: object
        self.step: int = 0

        self.call_reqreserve= [[]]
        self.play_time: float = 0
        self.episode = 0
        self.is_done = False

        for i in range(0, env_.elevator_count):
            c = Lift(self.scr, i, self)
            self.lifts.append(c)

        self.rest_passenger = self._env.passenger
        self.dest_passenger = 0
        self.simulation_time = 0
        self.add_passenger = 0
        self.play_time = 0
        self.episode = self.episode + 1

        SCREEN_WIDTH, SCREEN_HEIGHT = self.scr.get_size()
        METERPERPIXEL = SCREEN_HEIGHT / (self._env.floors + 1) / self._env.height
        LIFT_WIDTH = (SCREEN_WIDTH / METERPERPIXEL / (self._env.elevator_count + 1))

        dist = SCREEN_WIDTH / (self._env.elevator_count + 1)
        rest = self._env.elevator_count % 2
        mok = self._env.elevator_count / 2

        for i in range(self._env.elevator_count):
            if i >= len(self.lifts):
                car = Lift(self)
                self.lifts.append(car)
            self.lifts[i].pos.x = dist * (i+1)
            self.lifts[i].init(i, random.randint(0, 3))

        for i in range(self._env.floors):
            if i >= len(self.floors):
                f = Floor(self.scr, self, i)
                self.floors.append(f)
            else:
                self.floors[i].init(i)

            if len(self.call_reqreserve) <= i:
                self.call_reqreserve.append([-1,-1])
            else:
                self.call_reqreserve[i] = [-1,-1]

        self.step = 0
        self.is_done = False

    def simulation_passenger(self):
        # 해당 step에서 지금 현재 운반중인 승객들이 총 승객의 수의 30% 이상 출연해 있는 상태면 더 만들진 말자
        if self.curr_passenger > self._env.passenger * 0.3:
            return
        new_passenger = random.randint(0, self.rest_passenger)

        # 1층에서 승객들이 나오게 변경
        floor_passenger = [0 for i in range(len(self.floors))]
        floor_passenger[0] = random.randint(0, (int)(new_passenger * 0.8))
        rest = new_passenger - floor_passenger[0]

        while rest > 0:
            floor = random.randint(0, len(self.floors)-1)
            passenger = random.randint(1, rest)
            rest -= passenger
            floor_passenger[floor] = passenger

        for i in range(len(self.floors)):

            if floor_passenger[i] > 0:
                # todo : 여기 뭐지?
                dest_list = self.floors[i].add_passenger(floor_passenger[i])
                self.add_passenger += floor_passenger[i]
                self.rest_passenger -= floor_passenger[i]

        self.simulation_time += 5

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
        else:
            raise NotImplementedError

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
        self.play_time += 0.1  # todo : 단위가 궁금!

        if self.step >= self._env.maxstep:
            self.is_done = True
            return

        self.simulation_passenger()
        self.update_lift()
        self.step += 1

    def render(self):

        self.scr.fill(BLACK)

        for floor in self.floors:
            floor.render()

        for lift in self.lifts:
            lift.render()

        text = "Deep Lift Ep:" + str(self.episode) + " Success:" + str(self.success_count) + " Step:" + str(self.step)
        DrawText(self.scr, 0, 0, text, GREEN)


class Floor:
    def __init__(self, scr, b: Building):
        self.scr = scr
        self.floor_no = 0
        self._building = b
        self.passenger_list = []


    def __init__(self, scr, b: Building, no: int):
        self.scr = scr
        self.floor_no = no
        self._building = b
        self.passenger_list = []
        self.init(no)

    def init(self, f: int):
        self.up_call: bool = False
        self.down_call: bool = False
        self.pos = Vector3(0, 0, 0)
        self.pos.y = self._building._env.height * (f + 1)
        self.passenger_list.clear()
        self.checkTime = self._building.play_time


    def is_call(self, direction: MoveState):
        if direction == MoveState.UP:
            return self.up_call
        elif direction == MoveState.DOWN:
            return self.down_call
        else:
            return False

    # todo : 사용 안하는것 같음??
    # def set_passengers(self, count: int):
    #     self.num_passenger = count
    #
    #     for i in range(count):
    #         p: Passenger = Passenger(self.floor_no, self.floor_no)
    #         while True:
    #             p.dest_floor = random.randint(0, 10)
    #             if p.dest_floor != p.start_floor:
    #                 break
    #         self.passenger_list.append(p)

    def update(self):
        # todo : play_time이 마지막 체크시점보다 더 지났을 경우라는건가?
        if self._building.play_time - self.checkTime > 1:
            self.chk_call_button()

    def chk_call_button(self):
        self.up_call = False
        self.down_call = False
        for p in self.passenger_list:
            if p.dest_floor > self.floor_no:
                self.up_call = True
            else:
                self.down_call = True
            if self.down_call and self.up_call:
                break
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
            self.passenger_list.append(p)
        self.chk_call_button()
        return dest_list

    def take_passenger(self, lift: Lift):
        idx = 0
        while idx < len(self.passenger_list):
            if lift.take_passenger(self.passenger_list[idx]):
                self.passenger_list.remove(idx)
            else:
                ++idx

    def set_button(self, direction: MoveState, on: bool):
        if on:
            self._building.call_request(self.floor_no, direction)

    def render(self):
        x, y = ToScreenPos(self.pos)
        pg.draw.line(self.scr, WHITE, [x, y], [SCREEN_WIDTH, y], 3)

        DrawText(self.scr, x + METER_PER_PIXEL, y - METER_PER_PIXEL, str(self.floor_no), BLUE)
        DrawText(self.scr, x + METER_PER_PIXEL, y + 3, str(len(self.passenger_list)), RED)

        if self.up_call:
            DrawText(self.scr, x + METER_PER_PIXEL * 3, y - METER_PER_PIXEL, '^', BLUE)

        if self.down_call:
            DrawText(self.scr, x + METER_PER_PIXEL * 3, y + 3, 'V', RED)


class VerticalLine(object):
    def __init__(self, scr, env, floor: int, x: int):
        self.scr = scr
        self.env = env
        self.floor = floor
        self.pos = Vector3(x, (floor + 1) * self.env.height, 0)
        self.on = False

    def init(self):
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
            DrawText(self.scr, x + METER_PER_PIXEL * 3, y + 3, 'O', RED)
