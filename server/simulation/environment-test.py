import numpy as np
import pybullet as p
import time
import pybullet_data
import random
import math

# Connect to the PyBullet GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the transparent plane URDF
groundUid = p.loadURDF("plane_transparent.urdf", [0, 0, 0])
p.changeVisualShape(groundUid, -1, rgbaColor=[0, 1, 0, 1])  # Green ground

# Load the drone model (update with your drone URDF path)
drone_urdf = "./urdf/quadroter.urdf"
drone_id = p.loadURDF(drone_urdf, basePosition=[5, 0, 1], useFixedBase=False)

# Parameters for circular flight
radius = 5           # Circle radius
altitude = 2         # Constant altitude
speed = 100       # Speed factor to control flight speed
orbit_speed = 100   # Orbit speed to move the center of the circle

# Function to create random trees in the environment
def create_random_trees(num_trees):
    tree_ids = []
    for _ in range(num_trees):
        x = random.uniform(-10, 10)
        y = random.uniform(-10, 10)
        z = 0.5
        tree_urdf = "./urdf/tree.urdf"  # Update with your tree URDF path
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
        thickness = 0.01  # Thin to make it flat
        
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
    x = center[0] + radius * math.cos(speed * current_time)
    y = center[1] + radius * math.sin(speed * current_time)
    z = altitude

    # Get current position and orientation of the drone
    pos, orn = p.getBasePositionAndOrientation(drone_id)

    # Apply a force to make the drone move towards the calculated position
    target_velocity = np.array([x - pos[0], y - pos[1], z - pos[2]]) 
    target_velocity *= 100.0
    p.applyExternalForce(drone_id, -1, forceObj=target_velocity, posObj=pos, flags=p.WORLD_FRAME)

    # Set up the camera behind and above the drone for a normal POV
    drone_pos, drone_orn = p.getBasePositionAndOrientation(drone_id)
    rotation_matrix = p.getMatrixFromQuaternion(drone_orn)

    # Camera position slightly behind and above the drone
    camera_position = [
        drone_pos[0] - 1.5 * rotation_matrix[0],  # Backward
        drone_pos[1] - 1.5 * rotation_matrix[3],  # Sideways
        drone_pos[2] + 1.0  # Above
    ]

    # Camera target directly in front of the drone
    camera_target = [
        drone_pos[0] + 0.5 * rotation_matrix[0],  # In front
        drone_pos[1] + 0.5 * rotation_matrix[3],  # In front
        drone_pos[2]  # At the same altitude
    ]

    # Get camera view and projection matrices
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_position,
        cameraTargetPosition=camera_target,
        cameraUpVector=[0, 0, 1]  # Upward direction
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=60.0,
        aspect=1.0,
        nearVal=0.1,
        farVal=100.0
    )

    # Capture the camera image
    img_width, img_height = 320, 240
    _, _, rgb_img, _, _ = p.getCameraImage(
        width=img_width,
        height=img_height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix
    )

    # (Optional) Process rgb_img here if needed, e.g., display or save

    # Delay for simulation timing
    time.sleep(1./240.)
