from enum import Enum
import time


class State(Enum):
    Ready = 1           # 문닫고 멈춰있는 상태
    NormalMove = 2      # 위아래 어느쪽이든 정상적인 이동상태
    Decelerate = 3      # 다음층에 멈추기 위한 감속상태
    MoveStop = 4        # 이동하는 중에 각층에 멈춤상태
    DoorOpening = 5     # 문열는중
    DoorOpened = 6      # 문열린 상태에서 승객내리고 타고
    DoorClosing = 7     # 문닫히는 동안.
    Accelate = 8        # 이동에 대한 가속상태
    Turn = 9
    End = 10


class Event(Enum):
    Call = 1                # 각층에서 호출이 왔을 경우.
    DecelerateStart = 2     # 이동중에 감속지점을 통과 했을때
    Arrived = 3             # 각 층에 도착했을때
    DoorOpenRequest = 4     # 문열기 요청
    DoorOpenEnd = 5         # 문열기 끝
    DoorCloseStart = 6      # 문닫기 시작
    DoorCloseEnd = 7        # 문닫기 끝
    AccelateEnd = 8         # 가속끝 정상속도 도달
    EmptyPassenger = 9      # 승객이없다. 전부 내렸다.
    End = 10


class FiniteState:
    def __init__(self, state: State):
        self.my_state = state
        self.transition: dict = {}      # dict[Event,finiteState]

    def add_transition(self, event: Event, out_state: State):
        self.transition[event] = out_state

    def del_transition(self, event: Event):
        del self.transition[event]

    def trans(self, input_event: Event):
        return self.transition.get(input_event)


class FSM:
    def __init__(self, state: State):
        self.curr_state = state
        self.state_dic: dict = {}
        self.transition_time: int = 0

    def add_transition(self, state: State, event: Event, out_state: State):
        if state not in self.state_dic:
            self.state_dic[state] = FiniteState(state)
        if out_state not in self.state_dic:
            self.state_dic[out_state] = FiniteState(out_state)
        in_state: FiniteState = self.state_dic[state]
        in_state.add_transition(event, out_state)
   
    def transition(self, event: Event):
        if self.curr_state in self.state_dic.keys():
            self.transition_time = time.time()
            new_state = self.state_dic[self.curr_state].trans(event)
            if new_state is not None:
                self.curr_state = new_state
            else:
                # raise ValueError('There is available next state in FSM.')
                pass
            return self.curr_state
        else:
            # todo : Is it ok?
            raise ValueError('Current FSM has a problem for setting current state.')
            return State.End

    def get_current_state(self):
        if self.curr_state not in self.state_dic:
            raise ValueError('Current FSM has a problem for setting current state.')
            return State.End
        return self.curr_state