from enum import Enum, auto

class RoverState(Enum):
    SELF_DIAGNOSE = auto()
    IDLE = auto()
    CHARGING = auto()
    MOVING = auto()
    COLLECTING_SAMPLE = auto()
    COMMUNICATING = auto()
    FINAL_STATE = auto()

class StateMachine:
    """
    Manages the operational states of the Mars Rover.
    I modeled this state machine directly from the hand-drawn State diagram.
    """
    def __init__(self):
        self._current_state = RoverState.SELF_DIAGNOSE
        print(f"[State] Rover initialized. Current State: {self._current_state.name}")

    @property
    def current_state(self) -> RoverState:
        return self._current_state

    def transition_to(self, new_state: RoverState, trigger: str):
        """
        Transitions the rover to a new state and logs the trigger.
        """
        old_state = self._current_state
        self._current_state = new_state
        print(f"[State Transition] Triggered by '{trigger}': {old_state.name} ---> {new_state.name}")
