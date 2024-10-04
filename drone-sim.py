import os
import time
import argparse
import numpy as np
import pybullet as p

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.VelocityAviary import VelocityAviary
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

DEFAULT_DRONE = DroneModel("cf2x")
DEFAULT_GUI = True
DEFAULT_RECORD_VIDEO = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = False
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 5
#DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False

def run(
        drone=DEFAULT_DRONE,
        gui=DEFAULT_GUI,
        record_video=DEFAULT_RECORD_VIDEO,
        plot=DEFAULT_PLOT,
        user_debug_gui=DEFAULT_USER_DEBUG_GUI,
        obstacles=DEFAULT_OBSTACLES,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        duration_sec=DEFAULT_DURATION_SEC,
        #output_folder=DEFAULT_OUTPUT_FOLDER,
        colab=DEFAULT_COLAB
        ):
    #### Initialize the simulation #############################
    INIT_XYZS = np.array([[0, 0, .1], [.3, 0, .1], [.6, 0, .1], [0.9, 0, .1]])
    INIT_RPYS = np.array([[0, 0, 0], [0, 0, np.pi/3], [0, 0, np.pi/4], [0, 0, np.pi/2]])
    
    #### Create the environment ################################
    env = VelocityAviary(drone_model=drone,
                         num_drones=4,
                         initial_xyzs=INIT_XYZS,
                         initial_rpys=INIT_RPYS,
                         physics=Physics.PYB,
                         neighbourhood_radius=10,
                         pyb_freq=simulation_freq_hz,
                         ctrl_freq=control_freq_hz,
                         gui=gui,
                         record=record_video,
                         obstacles=obstacles,
                         user_debug_gui=user_debug_gui
                         )

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()
    DRONE_IDS = env.getDroneIds()

    #### Compute number of control steps in the simulation ######
    NUM_WP = control_freq_hz * duration_sec
    wp_counters = np.zeros(4, dtype=int)

    #### Initialize the velocity target ########################
    TARGET_VEL = np.zeros((4, NUM_WP, 4))
    for i in range(NUM_WP):
        TARGET_VEL[0, i, :] = [-0.5, 1, 0, 0.99] if i < (NUM_WP / 8) else [0.5, -1, 0, 0.99]
        TARGET_VEL[1, i, :] = [0, 1, 0, 0.99] if i < (NUM_WP / 8 + NUM_WP / 6) else [0, -1, 0, 0.99]
        TARGET_VEL[2, i, :] = [0.2, 1, 0.2, 0.99] if i < (NUM_WP / 8 + 2 * NUM_WP / 6) else [-0.2, -1, -0.2, 0.99]
        TARGET_VEL[3, i, :] = [0, 1, 0.5, 0.99] if i < (NUM_WP / 8 + 3 * NUM_WP / 6) else [0, -1, -0.5, 0.99]

    #### Run the simulation ####################################
    action = np.zeros((4, 4))
    START = time.time()
    
    for i in range(int(duration_sec * control_freq_hz)):
        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = env.step(action)

        #### Compute control for the current way point #############
        for j in range(4):
            action[j, :] = TARGET_VEL[j, wp_counters[j], :] 

        #### Go to the next way point and loop #####################
        for j in range(4):
            wp_counters[j] = (wp_counters[j] + 1) % NUM_WP

        #### Printout ##############################################
        env.render()

        #### Sync the simulation ###################################
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)

    #### Close the environment #################################
    env.close()

if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Velocity control example using VelocityAviary')
    parser.add_argument('--drone', default=DEFAULT_DRONE, type=DroneModel, help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--gui', default=DEFAULT_GUI, type=str2bool, help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video', default=DEFAULT_RECORD_VIDEO, type=str2bool, help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot', default=DEFAULT_PLOT, type=str2bool, help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui', default=DEFAULT_USER_DEBUG_GUI, type=str2bool, help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--obstacles', default=DEFAULT_OBSTACLES, type=str2bool, help='Whether to add obstacles to the environment (default: False)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ, type=int, help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz', default=DEFAULT_CONTROL_FREQ_HZ, type=int, help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec', default=DEFAULT_DURATION_SEC, type=int, help='Duration of the simulation in seconds (default: 5)', metavar='')
    parser.add_argument('--colab', default=DEFAULT_COLAB, type=str2bool, help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()
    
    run(**vars(ARGS))
