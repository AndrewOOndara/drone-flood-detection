import matplotlib.pyplot as plt
import numpy as np
from PyFlyt.core import Aviary

# Function to calculate distances to objects
def calculate_proximity(drone_position, objects_positions):
    distances = np.linalg.norm(objects_positions - drone_position, axis=1)
    return distances

# Starting position and orientations for the drone
start_pos = np.array([[0.0, 0.0, 1.0]])  # Starting position (x, y, z)
start_orn = np.array([[0.0, 0.0, 0.0]])  # Starting orientation (roll, pitch, yaw)

# Environment setup with the drone's camera
env = Aviary(
    start_pos=start_pos,
    start_orn=start_orn,
    render=True,
    drone_type="quadx",
    drone_options=dict(
        use_camera=False,  # Disable camera for proximity sensing example
    ),
)

# Define some random objects in the environment
num_objects = 5
 # Assuming add_obstacles is a method to place objects in the env

# Set to velocity control
env.set_mode(6)

# Simulate for 100 steps
for i in range(100):
    env.step()
    
    # Get the current position of the drone
    drone_position = env.drones[0].position  # Assuming position returns the current position of the drone
    
    # Calculate distances to each object
    distances = calculate_proximity(drone_position, objects_positions)
    
    # Detect proximity
    proximity_threshold = 1.5  # Set a threshold for proximity detection
    close_objects = distances[distances < proximity_threshold]
    
    # Print the number of close objects
    print(f"Step {i}: Close objects detected: {len(close_objects)}")

plt.show()
