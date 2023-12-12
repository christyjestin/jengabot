from enum import Enum
from pydrake.all import (
    BasicVector,
    AbstractValue,
    LeafSystem,
    PiecewisePolynomial,
    PiecewisePose,
    RigidTransform
)

from constants import *
from trajectory import *

# Conceptually, we want to push out the block part way, then grab it from the other end, 
# bring it up to the top, put it down, and adjust until it's in place

# State sequence starting and ending at home w/ closed grip:
# POSITION_FOR_PUSH -> PUSH_BLOCK -> RETRACT -> MOVE_TO_SIDE -> OPEN_GRIP -> POSITION_FOR_PULL -> 
# CLOSE_GRIP -> PULL_BLOCK -> GO_TO_TOP -> POSITION_FOR_DROP -> OPEN_GRIP -> MOVE_TO_SIDE -> 
# CLOSE_GRIP -> POSITION_FOR_ADJUST -> ADJUST_BLOCK -> GO_HOME
class PlannerState(Enum):
    # Relocate Type: Diff IK, no collisions, path doesn't matter
    POSITION_FOR_PUSH = 0
    POSITION_FOR_PULL = 1
    POSITION_FOR_DROP = 2 # Line up the block at the top
    POSITION_FOR_ADJUST = 3
    GO_TO_TOP = 4 # Go straight up
    MOVE_TO_SIDE = 5 # Go out towards side so that you can open and close gripper freely
    GO_HOME = 6
    # Push/Pull Type: Diff IK, no collisions, path matters
    PUSH_BLOCK = 7
    RETRACT = 8 # inverse of PUSH_BLOCK
    PULL_BLOCK = 9
    ADJUST_BLOCK = 10 # Adjust block on top of stack
    # Gripper Type: Open or close gripper without moving iiwa
    OPEN_GRIP = 11
    CLOSE_GRIP = 12
    # Miscellaneous
    IDLE = 13 # Don't move at all

def get_state(context, index):
    return context.get_abstract_state(index).get_value()

def set_state(state, index, value):
    state.get_mutable_abstract_state(index).set_value(value)

class Planner(LeafSystem):
    def __init__(self, plant):
        LeafSystem.__init__(self)
        # input ports
        self.iiwa_position_index = self.DeclareVectorInputPort("iiwa_position", IIWA_NUM_POSITIONS).get_index()
        self.wsg_state_index = self.DeclareVectorInputPort("wsg_state", 2).get_index()
        self.body_poses_index = self.DeclareAbstractInputPort("body_poses", AbstractValue.Make([RigidTransform()])).get_index()

        # gripper body pose is necessary for planning
        self.gripper_body_index = plant.GetBodyByName("body").index()

        # state variables
        self.planner_state_index = int(self.DeclareAbstractState(AbstractValue.Make(PlannerState.IDLE)))
        self.X_G_trajectory_index = int(self.DeclareAbstractState(AbstractValue.Make(PiecewisePose.MakeLinear([0, 1], 
                                                                                     [HOME_POSE, HOME_POSE]))))
        self.wsg_trajectory_index = int(self.DeclareAbstractState(AbstractValue.Make(PiecewisePolynomial([0]))))

        # output ports
        self.DeclareAbstractOutputPort("X_WG", lambda: AbstractValue.Make(RigidTransform()), self.CalcGripperPose)
        self.DeclareVectorOutputPort("wsg_position", BasicVector([0]), self.CalcWsgPosition)
        self.DeclareAbstractOutputPort("use_robot_state", lambda: AbstractValue.Make(False), self.CalcUseRobotState)

        # handler for running updates
        self.DeclarePeriodicUnrestrictedUpdateEvent(0.1, 0.0, self.Update)
        self.no_plan = True

    def Update(self, context, state):
        if self.no_plan:
            self.no_plan = False
            self.Plan(context, state)
            return
        if not get_state(context, self.X_G_trajectory_index).is_time_in_range(context.get_time()):
            self.Plan(context, state)

    def Plan(self, context, state):
        current_time = context.get_time()

        block = (14, 0)
        times, poses, wsg_positions = construct_trajectory(current_time, block)
        set_state(state, self.X_G_trajectory_index, PiecewisePose.MakeLinear(times, poses))
        set_state(state, self.wsg_trajectory_index, PiecewisePolynomial.FirstOrderHold(times, wsg_positions))

    def CalcGripperPose(self, context, output):
        output.set_value(get_state(context, self.X_G_trajectory_index).GetPose(context.get_time()))

    def CalcWsgPosition(self, context, output):
        output.SetFromVector(get_state(context, self.wsg_trajectory_index).value(context.get_time()))

    def CalcUseRobotState(self, context, output):
        pass