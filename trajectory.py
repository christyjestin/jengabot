import numpy as np
from pydrake.all import (
    RigidTransform,
    RotationMatrix,
    RollPitchYaw
)

# import constants defining the block geometry and push behavior
from constants import *

PUSH_POINT_DISTANCE_FROM_BODY = 0.1181 # (meters) distance the gripper needs to push
GRAB_POINT_DISTANCE_FROM_BODY = 0.108  # meters

# Most example code assumes the iiwa base is at the origin, but ours is different
IIWA_BASE_TRANSLATION = np.array([0.35, 0.5, 0.015])

# The rotatiion to reach the home position from the iiwa based position
HOME_ROTATION = RotationMatrix([
    [0.7114733527908449, -0.7027128539656081, -0.0005595891930164908],
    [-0.702713076773554, -0.711473127205294, -0.0005665652348023884],
    [-4.857403548424557e-17, 0.0007963267107334358, -0.999999682931835]
])
# Translate in order to reach the home position.
HOME_TRANSLATION = np.array([-0.053822409986524454, 0.6596538706985489, 0.7704539062251178])
HOME_POSE = RigidTransform(HOME_ROTATION, HOME_TRANSLATION)

# Gripper distances for open and close
CLOSED_EMPTY_GRIPPER = 0 # (m)
OPEN_GRIPPER = 0.1 # (m)

# Useful orientations
FACING_NEG_Y = RollPitchYaw(0, 0, np.pi).ToRotationMatrix()
FACING_NEG_X = RollPitchYaw(0, 0, 0.5 * np.pi).ToRotationMatrix()
# FACING_POS_X = RollPitchYaw(0, 0, yaw).ToRotationMatrix()
# FACING_POS_X = RollPitchYaw(0, 0, yaw).ToRotationMatrix()

# motion parameters
PUSH_FRACTION = 0.2

# shift world coordinate position into base coordinates for Iiwa Diff IK
def in_base_frame(point):
    return point - IIWA_BASE_TRANSLATION

def construct_trajectory(current_time, block):
    # The layer variable indexes along the z axis. 
    layer, index = block

    # Every other layer is a "layer_in_x_direction" because of the arrangement of the Jenga tower.
    layer_in_x_direction = layer % 2 == 0

    # shift_direction is in y if the current layer is "layer_in_x_direction" and vice versa
    # think of this as how to go from near to far end of the block
    shift_direction = np.array([0, 1, 0]) if layer_in_x_direction else np.array([1, 0, 0])
    
    # Compute the z position of the gripper baesd on the block height, current layer, and table height.
    z = BLOCK_HEIGHT * (layer + 0.5) + TABLE_HEIGHT
    # center of the block's smaller face from the side closer to the Iiwa
    if layer_in_x_direction:
        near_end = np.array([BLOCK_WIDTH * (index - 1), BLOCK_LENGTH / 2, z])
    else:
        near_end = np.array([BLOCK_LENGTH / 2, BLOCK_WIDTH * (index - 1), z])
    far_end = near_end + shift_direction * -BLOCK_LENGTH    
    
    PUSH_ORIENTATION = FACING_NEG_Y if layer_in_x_direction else FACING_NEG_X
    push_orienting_point = near_end + shift_direction * 0.15 # point farther away from the stack to get in position
    push_start_point = near_end + shift_direction * PUSH_POINT_DISTANCE_FROM_BODY
    push_end_point = push_start_point - shift_direction * PUSH_FRACTION * BLOCK_LENGTH
    side_offset = 1.5 * BLOCK_WIDTH + 0.2
    side_point = np.array([side_offset, BLOCK_LENGTH / 2, z]) if layer_in_x_direction else np.array([BLOCK_LENGTH / 2, side_offset, z])
    
    
    # each tuple is duration (time since previous step), pose, gripper position
    trajectory = [
        (0, HOME_POSE, CLOSED_EMPTY_GRIPPER), # start
        # Get in position
        (0.2, RigidTransform(PUSH_ORIENTATION, push_orienting_point), CLOSED_EMPTY_GRIPPER),
        # Push block part of the way out
        (0.2, RigidTransform(PUSH_ORIENTATION, push_start_point), CLOSED_EMPTY_GRIPPER), 
        (0.2, RigidTransform(PUSH_ORIENTATION, push_end_point), CLOSED_EMPTY_GRIPPER),
        (0.2, RigidTransform(PUSH_ORIENTATION, push_start_point), CLOSED_EMPTY_GRIPPER), # retract
        (0.2, RigidTransform(PUSH_ORIENTATION, side_point), CLOSED_EMPTY_GRIPPER), # move to side
        # (1, HOME_POSE, CLOSED_EMPTY_GRIPPER), # move forward
        # (1, HOME_POSE, OPEN_GRIPPER), # open gripper
        # (1, HOME_POSE, CLOSED_EMPTY_GRIPPER), # move in for grip
        # (1, HOME_POSE, CLOSED_EMPTY_GRIPPER), # close gripper
        # (1, HOME_POSE, CLOSED_EMPTY_GRIPPER), # pull out
        # (1, HOME_POSE, CLOSED_EMPTY_GRIPPER), # move up
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

    durations, poses, wsg_positions = zip(*trajectory)
    poses = [RigidTransform(pose.rotation(), in_base_frame(pose.translation())) for pose in poses]
    return current_time + np.cumsum(durations), poses, np.array(wsg_positions).reshape(1, -1)