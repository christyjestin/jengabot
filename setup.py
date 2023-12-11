from constants import *
from manipulation.station import load_scenario, MakeHardwareStation
from pydrake.all import (
    DiagramBuilder,
    PortSwitch,
    Simulator,
    MeshcatVisualizer,

)
from manipulation.scenarios import AddIiwaDifferentialIK

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

def make_station(meshcat):
    meshcat.Delete()
    scenario = construct_scenario()
    builder = DiagramBuilder()
    station = builder.AddSystem(MakeHardwareStation(scenario, meshcat))
    plant = station.GetSubsystemByName("plant")
    return station, plant, builder

def setup_planner(builder, station, plant):
    planner = builder.AddSystem(Planner(plant))
    builder.Connect(
        station.GetOutputPort("body_poses"), planner.GetInputPort("body_poses")
    )
    builder.Connect(
        station.GetOutputPort("wsg.state_measured"),
        planner.GetInputPort("wsg_state"),
    )
    builder.Connect(
        station.GetOutputPort("iiwa.position_measured"),
        planner.GetInputPort("iiwa_position"),
    )
    robot = station.GetSubsystemByName(
        "iiwa.controller"
    ).get_multibody_plant_for_control()
    return planner, robot

def setup_controller(builder, robot, planner, station):
    diff_ik = AddIiwaDifferentialIK(builder, robot)
    builder.Connect(planner.GetOutputPort("X_WG"), diff_ik.get_input_port(0))
    builder.Connect(
        station.GetOutputPort("iiwa.state_estimated"),
        diff_ik.GetInputPort("robot_state"),
    )
    builder.Connect(
        planner.GetOutputPort("reset_diff_ik"),
        diff_ik.GetInputPort("use_robot_state"),
    )
    builder.Connect(
        planner.GetOutputPort("wsg_position"),
        station.GetInputPort("wsg.position"),
    )

    # The DiffIK and the direct position-control modes go through a PortSwitch
    switch = builder.AddSystem(PortSwitch(7))

    builder.Connect(
        diff_ik.get_output_port(), switch.DeclareInputPort("diff_ik")
    )
    builder.Connect(
        planner.GetOutputPort("iiwa_position_command"),
        switch.DeclareInputPort("position"),
    )
    builder.Connect(
        switch.get_output_port(), station.GetInputPort("iiwa.position")
    )
    builder.Connect(
        planner.GetOutputPort("control_mode"),
        switch.get_port_selector_input_port(),
    )
    return builder

def setup_simulation(meshcat, builder, station):
    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, station.GetOutputPort("query_object"), meshcat
    )
    diagram = builder.Build()

    simulator = Simulator(diagram)

    simulator.AdvanceTo(0.1)
    meshcat.Flush()  # Wait for the large object meshes to get to meshcat.

    simulator.set_target_realtime_rate(1.0)
    meshcat.AddButton("Stop Simulation", "Escape")
    print("Press Escape to stop the simulation")
    while meshcat.GetButtonClicks("Stop Simulation") < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + 2.0)
    meshcat.DeleteButton("Stop Simulation")
    return visualizer, diagram, simulator
