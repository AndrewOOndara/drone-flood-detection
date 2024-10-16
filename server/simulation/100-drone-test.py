import pybullet as p
import pybullet_data
import numpy as np
import time
import os

# Connect to PyBullet and set up the simulation
physics_client = p.connect(p.GUI)  # Use p.DIRECT for headless simulation
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Path for plane URDF
p.setGravity(0, 0, -9.8)  # Set gravity in the simulation

# Load ground plane
plane_id = p.loadURDF("plane.urdf")

# Set up parameters for simulation
NUM_DRONES = 100  # Number of drones
VELOCITY_MULTIPLIER = 3  # Controls how fast drones move
SIMULATION_DURATION_SEC = 20
SIMULATION_FREQ_HZ = 240
CONTROL_FREQ_HZ = 48
SPAWN_RADIUS = 3  # The radius within which drones will be spawned

# Time step settings
p.setTimeStep(1.0 / SIMULATION_FREQ_HZ)

# Path to the custom drone URDF (replace with your URDF file path)
DRONE_URDF_PATH = "/Users/andrewondara/drone-flood-detection/server/urdf/cf2.urdf"  # Ensure this is the correct path to your saved URDF file

# Spawn drones in a small area around a central point (closely packed)
drone_ids = []
initial_positions = np.random.uniform(low=-SPAWN_RADIUS, high=SPAWN_RADIUS, size=(NUM_DRONES, 3))
initial_positions[:, 2] = 1  # Set initial height (Z-axis) to 1 meter

for i in range(NUM_DRONES):
    drone_id = p.loadURDF(DRONE_URDF_PATH, initial_positions[i])
    drone_ids.append(drone_id)

# Set random target velocities for drones
target_velocities = np.random.uniform(-1, 1, (NUM_DRONES, SIMULATION_DURATION_SEC * CONTROL_FREQ_HZ, 3)) * VELOCITY_MULTIPLIER

# Simulation loop
for step in range(int(SIMULATION_DURATION_SEC * SIMULATION_FREQ_HZ)):
    for i in range(NUM_DRONES):
        # Get random velocities for the current step
        velocity = target_velocities[i, step % (CONTROL_FREQ_HZ * SIMULATION_DURATION_SEC), :]

        # Apply velocity to the drone (acting like a simple flying object)
        p.resetBaseVelocity(drone_ids[i], linearVelocity=velocity)

    # Step the simulation forward
    p.stepSimulation()

    # Optional: Slow down the simulation to real-time speed
    time.sleep(1.0 / SIMULATION_FREQ_HZ)

# Disconnect the PyBullet client after simulation
p.disconnect()
