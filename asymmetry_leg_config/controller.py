# Make sure to have the add-on "ZMQ remote API"
# running in CoppeliaSim
#
# All CoppeliaSim commands will run in blocking mode (block
# until a reply from CoppeliaSim is received). For a non-
# blocking example, see simpleTest-nonBlocking.py
#
# if there is an error on dependencies
# python3 -m pip install coppeliasim-zmqremoteapi-client

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import os
import random
import math
from GaitPlanner import GaitPlanner
from neural_control import CPG, MotorMapper

def main():
    print()
    print('---------- Program started ----------')

    # Define the path to your CoppeliaSim scene file
    scene_file_path = os.path.abspath("/home/binggwong/git/StarfishBot/scenes/Asymmetry_Leg_gait.ttt")

    # Create a client to connect to CoppeliaSim
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.addLog(sim.verbosity_scriptinfos, '---------- Simulation started ----------')

    # Open the specified scene
    try:
        print(f"Loading scene: {scene_file_path}")
        sim.loadScene(scene_file_path)
    except Exception as e:
        print(f"Failed to load scene: {e}")
        return

    # Get object handles
    robot_handle = sim.getObject('/hexapod')
    graph_handle = sim.getObject('/graph')

    # Leg config
    num_leg = 6
    num_joints_per_leg = 3
    num_joints = num_leg*num_joints_per_leg
    joints_bias = [0, -0.52, 2.09]
    leg_angle_handle = []
    joint_handles = []
    leg_angle = []
    for l in range(num_leg):
        leg_name = ':/Revolute_joint[' + str(l) + ']'
        leg_angle_handle.append(sim.getObject(leg_name))
        leg_angle.append(round(math.degrees(sim.getJointPosition(leg_angle_handle[l]))))
        for j in range(num_joints_per_leg):
            joint_name = 'joint' + str(j+1) + '_' + str(l)
            if j == 0:
                joint_handles.append(sim.getObject(':/'+joint_name))
            else:
                joint_handles.append(sim.getObject(':/'+joint_name))
            # print(joint_name)
            # print(len(joint_handles))
    # print('leg_angle: ', leg_angle)
    # sim.addLog(sim.verbosity_scriptinfos, 'leg_angle: {0}'.format(leg_angle))
    
    # Example usage
    default_angles = [0, 60, 120, 180, 240, 300]
    angles = default_angles  # Hexapod leg angles in degrees
    # angles = [0, 60, 120, 180, 240, 300] # symmetry
    angles = [0, 30, 60, 80, 120, 270]  # asymmetry
    for i, leg_handle in enumerate(leg_angle_handle):
        if angles[i] > 180:
            sim.setJointTargetPosition(leg_handle, math.radians(angles[i]-360))
        else:
            sim.setJointTargetPosition(leg_handle, math.radians(angles[i]))
    max_timesteps = 20
    gait_planner_hexa = GaitPlanner(angles, max_legs_lifted=2, lifting_period=40)
    # Create an instance of RandomPolygonAnimation with desired parameters

    cpg = CPG()
    motor_mapping = MotorMapper(6, 3)
    motor_mapping.set_weight_leg([0, -0.8, 0.0])

    # f_x = sim.addGraphStream(graph_handle, 'F_x', 'N', 0, [1, 0, 0])

    # Run a simulation in stepping mode
    sim.setStepping(True)
    sim.startSimulation()
    while (t := sim.getSimulationTime()) < 5:

        # print('timestep: ', i)
        gait_plan, lifted_legs = gait_planner_hexa.gait_planner(angles)
        print("Gait Plan (Timestep, Leg Lifting Order):", gait_plan)
        # print("Remaining Lifted Legs (with timers):", lifted_legs)

        cpg.step()
        cpg_out = cpg.get_CPG_output()
        motor_commands = motor_mapping.map_to_motors(cpg_out, gait_plan, joints_bias)

        for i in range(num_joints):
            sim.setJointTargetPosition(joint_handles[i], motor_commands[i])

        # Log force magnitude
        force_magnitude = random.uniform(0,1)
        # s = f'Simulation time: {t:.2f} [s], Force magnitude: {force_magnitude:.2f} N'
        # print(s)
        # sim.addLog(sim.verbosity_scriptinfos, s)

        # Update the graph
        # sim.setGraphStreamValue(graph_handle, f_x, force_vector[0])

        sim.step()  # triggers next simulation step

    for i, leg_handle in enumerate(leg_angle_handle):
        if angles[i] > 180:
            sim.setJointTargetPosition(leg_handle, math.radians(default_angles[i]-360))    
        else:
            sim.setJointTargetPosition(leg_handle, math.radians(default_angles[i]))

    sim.stopSimulation()


    print('---------- Program ended ----------')
    print()
    sim.addLog(sim.verbosity_scriptinfos, '---------- Simulation ended ----------')

if __name__ == "__main__":
    main()