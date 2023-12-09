from constants import *
from manipulation.station import load_scenario

def make_stack():
    output = ''
    for i in range(NUM_LAYERS):
        # alternate block orientations
        layer_in_x_direction = i % 2 == 0
        for j in range(NUM_BLOCKS_PER_LAYER):
            x = (BLOCK_WIDTH * (j - 1)) if layer_in_x_direction else 0
            y = 0 if layer_in_x_direction else (BLOCK_WIDTH * (j - 1))
            z = BLOCK_HEIGHT * (i + 0.5) + TABLE_HEIGHT
            orientation = "{ deg: [0, 0, 0] }" if layer_in_x_direction else "{ deg: [0, 0, 90] }"
            output += f"""
- add_model:
    name: block_{i * 3 + j}
    file: package://jengabot/models/jenga_block.sdf
    default_free_body_pose:
        jenga_block_link:
            translation: [{x}, {y}, {z}]
            rotation: !Rpy {orientation}"""
    return output

def construct_scenario():
    scenario_data = """
directives:
- add_model:
    name: table_top
    file: package://jengabot/models/table_top.sdf
- add_weld:
    parent: world
    child: table_top::table_top_center
- add_model:
    name: iiwa
    file: package://drake/manipulation/models/iiwa_description/iiwa7/iiwa7_no_collision.sdf
    default_joint_positions:
        iiwa_joint_1: [-1.57]
        iiwa_joint_2: [0.1]
        iiwa_joint_3: [0]
        iiwa_joint_4: [-1.2]
        iiwa_joint_5: [0]
        iiwa_joint_6: [ 1.6]
        iiwa_joint_7: [0]
- add_weld:
    parent: table_top_link
    child: iiwa::iiwa_link_0
    X_PC:
        translation: [0.35, 0.5, 0.015]
- add_model:
    name: wsg
    file: package://drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf
- add_weld:
    parent: iiwa::iiwa_link_7
    child: wsg::body
    X_PC:
        translation: [0, 0, 0.09]
        rotation: !Rpy { deg: [90, 0, 90]}
"""

    driver_data = """
model_drivers:
    iiwa: !IiwaDriver
      hand_model_name: wsg
    wsg: !SchunkWsgDriver {}
"""

    scenario_data += make_stack()
    scenario_data += driver_data
    scenario = load_scenario(data = scenario_data)
    return scenario