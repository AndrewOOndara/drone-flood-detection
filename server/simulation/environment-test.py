import numpy as np
import pybullet as p
import pybullet_data
import time
import random
import math
from rrt import apply_motion_planning  # Import the motion planning function

# Connect to the PyBullet GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the transparent plane URDF
groundUid = p.loadURDF("plane_transparent.urdf", [0, 0, 0])
p.changeVisualShape(groundUid, -1, rgbaColor=[0, 1, 0, 1])  # Green ground

# Load the drone model (update with your drone URDF path)
drone_urdf = "./urdf/quadroter.urdf"
drone_id = p.loadURDF(drone_urdf, basePosition=[5, 0, 1], useFixedBase=False)

# Check if the drone was loaded correctly
if drone_id < 0:
    raise ValueError("Failed to load the drone model. Check the URDF path and file integrity.")

# Short delay after loading the drone
time.sleep(1)

# Parameters for circular flight
radius = 5
altitude = 2
speed = 100
orbit_speed = 100

# Function to create random trees in the environment
def create_random_trees(num_trees):
    tree_ids = []
    for _ in range(num_trees):
        x = random.uniform(-10, 10)
        y = random.uniform(-10, 10)
        z = 0.5
        tree_urdf = "./urdf/tree.urdf"
        tree_id = p.loadURDF(tree_urdf, basePosition=[x, y, z])
        tree_ids.append(tree_id)
    return tree_ids

# Function to create random flat flooded areas
def create_random_flooded_areas(num_areas):
    flood_ids = []
    for _ in range(num_areas):
        x = random.uniform(-10, 10)
        y = random.uniform(-10, 10)
        length = random.uniform(1, 3)
        width = random.uniform(1, 3)
        thickness = 0.01
        
        col_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[length / 2, width / 2, thickness / 2])
        visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[length / 2, width / 2, thickness / 2], rgbaColor=[0, 0, 1, 0.8])
        
        flood_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col_shape, baseVisualShapeIndex=visual_shape, basePosition=[x, y, thickness / 2])
        flood_ids.append(flood_id)
    return flood_ids

# Create random trees and flooded areas
num_trees = 20
trees = create_random_trees(num_trees)
num_flooded_areas = 50
flooded_areas = create_random_flooded_areas(num_flooded_areas)

# Combine obstacles as a list of positions for motion planning
obstacles = [p.getBasePositionAndOrientation(tree)[0] for tree in trees] + \
            [p.getBasePositionAndOrientation(flood)[0] for flood in flooded_areas]

# Time tracking for circular motion
start_time = time.time()

# Main simulation loop
while True:
    p.stepSimulation()
    current_time = time.time() - start_time

    # Update center position to create an orbiting effect
    center_x = math.cos(orbit_speed * current_time) * 3
    center_y = math.sin(orbit_speed * current_time) * 3
    center = [center_x, center_y, altitude]

    # Calculate target position for the drone in a circular path around the moving center
    target_x = center[0] + radius * math.cos(speed * current_time)
    target_y = center[1] + radius * math.sin(speed * current_time)
    target_z = altitude
    target_position = [target_x, target_y, target_z]

    # Use motion planning to move towards the target while avoiding obstacles
    apply_motion_planning(drone_id, target_position, obstacles, speed=50)

    # Delay for simulation timing
    time.sleep(1./240.)
