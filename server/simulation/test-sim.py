import matplotlib.pyplot as plt
import numpy as np
from PyFlyt.core import Aviary

# Starting position and orientation for the drone
start_pos = np.array([[0.0, 0.0, 1.0]])  # (x, y, z)
start_orn = np.array([[0.0, 0.0, 0.0]])  # (roll, pitch, yaw)

# Environment setup with the drone
env = Aviary(
    start_pos=start_pos,
    start_orn=start_orn,
    render=True,
    drone_type="quadx",
    drone_options=dict(
        use_camera=False,  # Disable camera for this example
    ),
)

# Set to velocity control mode
env.set_mode(6)

# Simulation parameters
num_steps = 100
drone_velocity = np.array([0.1, 0.0, 0.0])  # Move right in the x-direction

# Simulate for the specified number of steps
for i in range(num_steps):
    # Update drone's position
    env.drones[0].velocity = drone_velocity  # Set the drone's velocity
    env.step()  # Advance the simulation


plt.show()
