import numpy as np
import pybullet as p
import pybullet_data
import time
import random
import math
from motion_planning import apply_motion_planning  # Import the motion planning function

class DroneSimulation:
    def __init__(self):
        # Initialize PyBullet and load resources
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Load environment elements
        self.load_ground()
        self.drone_id = self.load_drone()
        
        # Parameters
        self.radius = 5
        self.altitude = 2
        self.speed = 100
        self.orbit_speed = 100

        # Initialize obstacles
        self.trees = self.create_random_trees(20)
        self.flooded_areas = self.create_random_flooded_areas(50)
        self.obstacles = [p.getBasePositionAndOrientation(tree)[0] for tree in self.trees] + \
                         [p.getBasePositionAndOrientation(flood)[0] for flood in self.flooded_areas]

        # Start the simulation timer
        self.start_time = time.time()

    def load_ground(self):
        # Load the transparent plane URDF
        ground_uid = p.loadURDF("plane_transparent.urdf", [0, 0, 0])
        p.changeVisualShape(ground_uid, -1, rgbaColor=[0, 1, 0, 1])  # Solid green color

    def load_drone(self):
        # Load the drone model
        drone_urdf = "./urdf/quadroter.urdf"
        drone_id = p.loadURDF(drone_urdf, basePosition=[5, 0, 1], useFixedBase=False)
        
        # Check if the drone was loaded correctly
        if drone_id < 0:
            raise ValueError("Failed to load the drone model. Check the URDF path and file integrity.")
        
        # Short delay after loading the drone
        time.sleep(1)
        return drone_id

    def create_random_trees(self, num_trees):
        tree_ids = []
        for _ in range(num_trees):
            x = random.uniform(-10, 10)
            y = random.uniform(-10, 10)
            z = 0.5
            tree_urdf = "./urdf/tree.urdf"
            tree_id = p.loadURDF(tree_urdf, basePosition=[x, y, z])
            tree_ids.append(tree_id)
        return tree_ids

    def create_random_flooded_areas(self, num_areas):
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

    def run_simulation(self):
        while True:
            p.stepSimulation()
            current_time = time.time() - self.start_time

            # Update center position to create an orbiting effect
            center_x = math.cos(self.orbit_speed * current_time) * 3
            center_y = math.sin(self.orbit_speed * current_time) * 3
            center = [center_x, center_y, self.altitude]

            # Calculate target position for the drone in a circular path around the moving center
            target_x = center[0] + self.radius * math.cos(self.speed * current_time)
            target_y = center[1] + self.radius * math.sin(self.speed * current_time)
            target_z = self.altitude
            target_position = [target_x, target_y, target_z]

            # Use motion planning to move towards the target while avoiding obstacles
            apply_motion_planning(self.drone_id, target_position, self.obstacles, speed=50)

            # Delay for simulation timing
            time.sleep(1./240.)

# Instantiate and run the drone simulation
if __name__ == "__main__":
    simulation = DroneSimulation()
    simulation.run_simulation()
