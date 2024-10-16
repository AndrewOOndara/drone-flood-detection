import numpy as np
import pybullet as p
import time
import pybullet_data
import random
from PyFlyt.core import Aviary

# Connect to the PyBullet GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the transparent plane URDF
groundUid = p.loadURDF("plane_transparent.urdf", [0, 0, 0])  # Green ground
p.changeVisualShape(groundUid, -1, rgbaColor=[0, 1, 0, 1])  # Solid green color

# Load the tree model (update with your tree URDF path)
tree_urdf = "./urdf/tree.urdf"

# Function to create random trees in the environment
def create_random_trees(num_trees):
    tree_ids = []
    for _ in range(num_trees):
        # Randomly generate a position for each tree
        x = random.uniform(-5, 5)
        y = random.uniform(-5, 5)
        z = 0.5  # Slightly above ground
        # Load the tree model
        tree_id = p.loadURDF(tree_urdf, basePosition=[x, y, z])
        tree_ids.append(tree_id)
    return tree_ids

# Create random trees
num_trees = 10  # Adjust the number of trees as needed
trees = create_random_trees(num_trees)

# the starting position and orientations for the drone
start_pos = np.array([[0.0, 0.0, 1.0]])
start_orn = np.array([[0.0, 0.0, 0.0]])

# environment setup for PyFlyt
env = Aviary(start_pos=start_pos, start_orn=start_orn, render=True, drone_type="quadx")

# set to position control (mode 7)
env.set_mode(7)

# Main simulation loop
for i in range(1000):  # Simulate for 1000 steps (1000/120 ~= 8 seconds)
    env.step()  # Step the PyFlyt environment

    # Optionally, you can also render images from the PyBullet camera
    width, height = 300, 300
    img = p.getCameraImage(width, height, renderer=p.ER_BULLET_HARDWARE_OPENGL)

# Disconnect from the simulation
p.disconnect()
