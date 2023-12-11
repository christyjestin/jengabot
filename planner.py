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
    
PUSH_DISTANCE = 0.1181
GRAB_DISTANCE = 0.108

def random_pose():
    index = np.random.randint(3)
    layer = np.random.randint(2) * 2 + 1
    # hit center of the block's smaller face from the side closer to the Iiwa
    target_x = BLOCK_LENGTH / 2 + PUSH_DISTANCE
    target_y = BLOCK_WIDTH * (index - 1)
    target_z = BLOCK_HEIGHT * (layer + 0.5) + TABLE_HEIGHT
    start_position = np.array([target_x, target_y, target_z])
    target_rotation = RollPitchYaw(0, 0, 0.5 * np.pi).ToRotationMatrix()
    print(layer, index, target_rotation, start_position)
    return RigidTransform(target_rotation, start_position)

IIWA_NUM_POSITIONS = 7
IIWA_BASE_TRANSLATION = np.array([0.35, 0.5, 0.015])
HOME_ROTATION = RotationMatrix([
    [0.9999996829318348, 0.00019052063137842194, -0.0007731999219133522],
    [0.0007963267107334455, -0.23924925335563643, 0.9709578572896668],
    [1.868506971441006e-16, -0.9709581651495911, -0.23924932921398248],
])
HOME_TRANSLATION = np.array([0.35037078321875903, 0.0343831919767536, 0.7093215789060889]) - IIWA_BASE_TRANSLATION
HOME_POSE = RigidTransform(HOME_ROTATION, HOME_TRANSLATION)

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
        
        
        self.gripper_body_index = plant.GetBodyByName("body").index()

        # state variables
        self.planner_state_index = int(self.DeclareAbstractState(AbstractValue.Make(PlannerState.IDLE)))
        self.X_G_trajectory_index = int(self.DeclareAbstractState(AbstractValue.Make(PiecewisePose.MakeLinear([0, 1], [HOME_POSE, HOME_POSE]))))
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
        current_pose = self.get_input_port(self.body_poses_index).Eval(context)[self.gripper_body_index]
        current_wsg = self.get_input_port(self.wsg_state_index).Eval(context)[0]
        current_time = context.get_time()
        
        sample_times = [current_time, current_time + 0.1, current_time + 1]
        target = random_pose() 
        target.set_translation(target.translation() - IIWA_BASE_TRANSLATION)
        poses = [current_pose, target, target]
        set_state(state, self.X_G_trajectory_index, PiecewisePose.MakeLinear(sample_times, poses))
        set_state(state, self.wsg_trajectory_index, PiecewisePolynomial([current_wsg]))

    def CalcGripperPose(self, context, output):
        output.set_value(get_state(context, self.X_G_trajectory_index).GetPose(context.get_time()))

    def CalcWsgPosition(self, context, output):
        output.SetFromVector(get_state(context, self.wsg_trajectory_index).value(context.get_time()))

    def CalcUseRobotState(self, context, output):
        pass