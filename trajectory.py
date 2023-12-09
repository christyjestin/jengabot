import numpy as np
from pydrake.all import (
    RigidTransform,
    RotationMatrix,
    RollPitchYaw
)

from constants import *

PUSH_DISTANCE = 0.1181
GRAB_DISTANCE = 0.108

IIWA_BASE_TRANSLATION = np.array([0.35, 0.5, 0.015])
HOME_ROTATION = RotationMatrix([
    [0.9999996829318348, 0.00019052063137842194, -0.0007731999219133522],
    [0.0007963267107334455, -0.23924925335563643, 0.9709578572896668],
    [1.868506971441006e-16, -0.9709581651495911, -0.23924932921398248],
])
HOME_TRANSLATION = np.array([0.35037078321875903, 0.0343831919767536, 0.7093215789060889]) - IIWA_BASE_TRANSLATION
HOME_POSE = RigidTransform(HOME_ROTATION, HOME_TRANSLATION)

CLOSED_EMPTY_GRIPPER = 0
OPEN_GRIPPER = 0.1

def construct_trajectory(current_time, block):
    # each tuple is duration (time since previous step), pose, gripper position
    trajectory = [
        (0, HOME_POSE, CLOSED_EMPTY_GRIPPER),
        (1, HOME_POSE, CLOSED_EMPTY_GRIPPER)
    ]
    # trajectory = [
    #     (0, , CLOSED_EMPTY_GRIPPER), # start
    #     (, , CLOSED_EMPTY_GRIPPER) # position for push
    #     () #
    # ]
    #     (PlannerState.POSITION_FOR_PUSH, )
    #     (PlannerState.PUSH_BLOCK)
    #     (PlannerState.RETRACT,)
    #     (PlannerState.MOVE_TO_SIDE)
    #     (PlannerState.OPEN_GRIP,)
    #     (PlannerState.POSITION_FOR_PULL)
    #     (PlannerState.CLOSE_GRIP,)
    #     (PlannerState.PULL_BLOCK,)
    #     (PlannerState.GO_TO_TOP,)
    #     (PlannerState.POSITION_FOR_DROP)
    #     (PlannerState.OPEN_GRIP,)
    #     (PlannerState.MOVE_TO_SIDE)
    #     (PlannerState.CLOSE_GRIP,)
    #     (PlannerState.POSITION_FOR_ADJUST)
    #     (PlannerState.ADJUST_BLOCK
    #     (PlannerState.GO_HOME
    # ]

    times, poses, wsg_positions = zip(*trajectory)
    return current_time + np.cumsum(times), list(poses), np.array(wsg_positions).reshape(1, -1)