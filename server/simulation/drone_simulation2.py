import numpy as np
import pybullet as p
import pybullet_data
import time
import random
import math
from motion_planning2 import apply_motion_planning  # Import the motion planning function

class Drone:
    def __init__(self, drone_id, start_position, radius, altitude, speed, orbit_speed):
        self.drone_id = drone_id
        self.position = start_position
        self.radius = radius
        self.altitude = altitude
        self.speed = speed
        self.orbit_speed = orbit_speed
        self.center_position = [random.uniform(-10, 10), random.uniform(-10, 10), self.altitude]  # Random center

    def update_position(self, current_time, drones, obstacles):
        # Calculate the drone's target position based on its orbit parameters
        center_x = math.cos(self.orbit_speed * current_time) * 3 + self.center_position[0]
        center_y = math.sin(self.orbit_speed * current_time) * 3 + self.center_position[1]
        center = [center_x, center_y, self.altitude]

        target_x = center[0] + self.radius * math.cos(self.speed * current_time)
        target_y = center[1] + self.radius * math.sin(self.speed * current_time)
        target_z = self.altitude
        target_position = [target_x, target_y, target_z]

        # Gather positions of other drones to avoid collisions
        other_drone_positions = [d.position for d in drones if d.drone_id != self.drone_id]
        nearby_obstacles = self.get_nearby_obstacles(obstacles, other_drone_positions)

        # If there are nearby obstacles or drones, update the target position to avoid them
        if nearby_obstacles:
            target_position = self.avoid_obstacles(target_position, nearby_obstacles)

        # Use motion planning to move towards the target while avoiding obstacles, including other drones
        apply_motion_planning(self.drone_id, target_position, nearby_obstacles, speed=50)

        # Update the drone's position
        self.position = target_position

    def get_nearby_obstacles(self, obstacles, other_drone_positions):
        # This function checks which obstacles are within the sensing radius of the drone
        nearby_obstacles = []
        
        # Check for nearby obstacles (trees, flooded areas, etc.)
        for obstacle in obstacles:
            distance = np.linalg.norm(np.array(self.position) - np.array(obstacle))
            if distance < 5:  # Fixed sensing radius for avoidance, you can adjust this value
                nearby_obstacles.append(obstacle)
        
        # Check for nearby drones (drone positions)
        for drone_pos in other_drone_positions:
            distance = np.linalg.norm(np.array(self.position) - np.array(drone_pos))
            if distance < 5:  # Fixed sensing radius for avoiding other drones
                nearby_obstacles.append(drone_pos)

        return nearby_obstacles

    def avoid_obstacles(self, target_position, nearby_obstacles):
        # If there are nearby obstacles or drones, adjust the target position to avoid them
        for obstacle in nearby_obstacles:
            # Simple avoidance: adjust target position to steer away from obstacle
            avoidance_vector = np.array(self.position) - np.array(obstacle)
            avoidance_vector /= np.linalg.norm(avoidance_vector)  # Normalize the vector
            avoidance_vector *= 0.5  # Scale the avoidance distance

            # Adjust the target position by steering away from the obstacle
            target_position = np.array(target_position) + avoidance_vector
            target_position = target_position.tolist()

        return target_position

    def get_position(self):
        return self.position


class DroneSimulation:
    def __init__(self, num_drones=1):
        # Initialize PyBullet and load resources
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Load environment elements
        self.load_ground()

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
        
        # Initialize drones
        self.drones = self.create_drones(num_drones)

        # Start the simulation timer
        self.start_time = time.time()

    def load_ground(self):
        # Load the transparent plane URDF
        ground_uid = p.loadURDF("plane_transparent.urdf", [0, 0, 0])
        p.changeVisualShape(ground_uid, -1, rgbaColor=[0, 1, 0, 1])  # Solid green color

    def load_drone(self, position):
        # Load the drone model at a specified position
        drone_urdf = "./urdf/quadroter.urdf"
        drone_id = p.loadURDF(drone_urdf, basePosition=position, useFixedBase=False)
        
        # Check if the drone was loaded correctly
        if drone_id < 0:
            raise ValueError("Failed to load the drone model. Check the URDF path and file integrity.")
        
        # Short delay after loading the drone
        time.sleep(1)
        return drone_id

    def create_drones(self, num_drones):
        drone_ids = []
        drones = []
        for i in range(num_drones):
            x_offset = random.uniform(-10, 10)
            y_offset = random.uniform(-10, 10)
            drone_id = self.load_drone([x_offset, y_offset, 1])
            
            # Initialize each drone with unique orbit parameters
            radius = random.uniform(3, 6)
            orbit_speed = random.uniform(1, 2)
            speed = random.uniform(50, 100)
            drone = Drone(drone_id, [x_offset, y_offset, 1], radius, 2, speed, orbit_speed)
            drones.append(drone)
            drone_ids.append(drone_id)
        return drones

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

            # Update position of each drone independently
            for drone in self.drones:
                drone.update_position(current_time, self.drones, self.obstacles)

            # Delay for simulation timing
            time.sleep(1./240.)

# Instantiate and run the drone simulation with 5 drones
if __name__ == "__main__":
    simulation = DroneSimulation(num_drones=5)
    simulation.run_simulation()
