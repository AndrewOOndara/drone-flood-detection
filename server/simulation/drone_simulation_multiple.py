import numpy as np
# import cv2
import pybullet as p
import pybullet_data
import os
import time
import random
import math
from rrt import apply_motion_planning  # Import the motion planning function

class DroneSimulation:
    def __init__(self):
        # Initialize PyBullet and load resources
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load environment elements
        self.load_ground()
        # self.drone_id = self.load_drone()
        self.num_drones = 2
        self.drone_ids = self.load_multiple_drones(self.num_drones)
        
        # Parameters
        self.radius = 5
        self.altitude = 2
        self.speed = 10
        self.orbit_speed = 10
        self.sensing_radius = 2  # Define sensing radius

        # Initialize obstacles
        self.trees = self.create_random_trees(20)
        self.flooded_areas = self.create_random_flooded_areas(50)
        self.obstacles = [p.getBasePositionAndOrientation(tree)[0] for tree in self.trees] + \
                         [p.getBasePositionAndOrientation(flood)[0] for flood in self.flooded_areas]

        # Start the simulation timer
        self.start_time = time.time()

        # Visualize sensing radius
        self.sensing_sphere_id = self.add_sensing_sphere(self.sensing_radius)

    def load_ground(self):
        ground_uid = p.loadURDF("plane_transparent.urdf", [0, 0, 0])
        p.changeVisualShape(ground_uid, -1, rgbaColor=[0, 1, 0, 1])  # Solid green color

    def load_drone(self):
        drone_urdf = "./urdf/quadroter.urdf"
        drone_id = p.loadURDF(drone_urdf, basePosition=[5, 0, 1], useFixedBase=False)
        if drone_id < 0:
            raise ValueError("Failed to load the drone model. Check the URDF path and file integrity.")
        time.sleep(1)
        return drone_id
    
    def load_multiple_drones(self, num_drones):
        drone_urdf = "./urdf/quadroter.urdf"
        drone_ids = []
        for _ in range(num_drones):
            x = random.uniform(-10, 10)
            y = random.uniform(-10, 10)
            z = 5
            drone_id = p.loadURDF(drone_urdf, basePosition=[x, y, z], useFixedBase=False)
            drone_ids.append(drone_id)
        return drone_ids


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

    def add_sensing_sphere(self, radius):
        # Creates a transparent sphere to represent the sensing radius
        col_sphere = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
        vis_sphere = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=[1, 0, 0, 0.2])  # Red with low opacity
        sphere_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col_sphere, baseVisualShapeIndex=vis_sphere, basePosition=[0, 0, 0])
        return sphere_id

    def update_sensing_sphere_position(self, drone_ids):
        # Update the sphere position to match the drone's current position
        for i in range(len(drone_ids)):
            drone_id = drone_ids[i]
            drone_position = p.getBasePositionAndOrientation(drone_id)[0]
            p.resetBasePositionAndOrientation(self.sensing_sphere_id, drone_position, [0, 0, 0, 1])

    def check_objects_in_radius(self, drone_ids):
        # Detect objects within the sensing radius
        for i in range(len(drone_ids)):
            drone_id = drone_ids[i]
            drone_position = p.getBasePositionAndOrientation(drone_id)[0]
            for obstacle_position in self.obstacles:
                distance = np.linalg.norm(np.array(drone_position) - np.array(obstacle_position))
                if distance <= self.sensing_radius:
                    print(f"Object detected within radius at position {obstacle_position}, distance {distance:.2f}")

    def run_simulation(self):
        while True:
            p.stepSimulation()
            current_time = time.time() - self.start_time

            # Update the sensing sphere to follow the drone
            self.update_sensing_sphere_position(self.drone_ids)
            self.check_objects_in_radius(self.drone_ids)  # Check for obstacles in radius

            # Calculate the center position for orbiting
            center_x = math.cos(self.orbit_speed * current_time) * 3
            center_y = math.sin(self.orbit_speed * current_time) * 3
            center = [center_x, center_y, self.altitude]

            # Calculate the target position for the drone in an orbit around the center
            target_x = center[0] + self.radius * math.cos(self.speed * current_time)
            target_y = center[1] + self.radius * math.sin(self.speed * current_time)
            target_position = [target_x, target_y, self.altitude]

            # Use motion planning to move towards the target while avoiding obstacles
            for i in range(len(self.drone_ids)):
                drone_id = self.drone_ids[i]
                apply_motion_planning(drone_id, target_position, self.obstacles, speed=50)

            # Delay for simulation timing
            time.sleep(1./240.)

# Instantiate and run the drone simulation
if __name__ == "__main__":
    simulation = DroneSimulation()
    simulation.run_simulation()
