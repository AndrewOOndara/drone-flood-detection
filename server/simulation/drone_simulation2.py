import numpy as np
import pybullet as p
import pybullet_data
import time
import random
import math
import cv2  
import os 
from scipy.spatial import KDTree
from PIL import Image
from motion_planning2 import apply_motion_planning  # Import the motion planning function
import matplotlib.pyplot as plt

import requests
import json


class Drone:
    def __init__(self, drone_id, start_position, radius, altitude, speed, sensing_radius):
        self.drone_id = drone_id
        self.position = start_position
        self.radius = radius
        self.altitude = altitude
        self.speed = speed
        self.center_position = [random.uniform(-10, 10), random.uniform(-10, 10), self.altitude]  # Random center
        self.sensing_radius = sensing_radius

        # self.waypoints = waypoints
        
        # Create the translucent red sphere to visualize the drone
        # self.sphere_id = self.create_red_sphere(start_position)

    # def create_red_sphere(self, position):
    #     # Create a translucent red sphere at the initial drone position
    #     sphere_radius = 0.5  # Radius of the sphere
    #     rgba_color = [1, 0, 0, 0.5]  # Red color with 50% transparency
    #     sphere_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=rgba_color)
        
    #     # Create a kinematic multi-body with no mass (i.e., just a visual object)
    #     sphere_id = p.createMultiBody(
    #         baseMass=0,  # No mass, making the object kinematic
    #         baseCollisionShapeIndex=-1,  # No collision for the sphere
    #         baseVisualShapeIndex=sphere_visual_shape,  # Visual shape is the red sphere
    #         basePosition=position  # Initial position of the sphere
    #     )
        
    #     return sphere_id

    def getID(self):
        return int(self.drone_id)

    def update_position(self, current_time, drones, obstacles, target_position):
       
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

        for o in nearby_obstacles:
            if o == target_position:
                print("COLLISION")
        
        # Now, update the position of the sphere to match the drone's position
        # p.resetBasePositionAndOrientation(self.sphere_id, self.position, [0, 0, 0, 1])  # No rotation
        # pos,ori = p.getBasePositionAndOrientation(self.sphere_id)

        print(f"Drone {self.drone_id} position: {self.position}")
        # print(f"Sphere {self.sphere_id} position: {pos}")


    def get_nearby_obstacles(self, obstacles, other_drone_positions):
        # This function checks which obstacles are within the sensing radius of the drone
        nearby_obstacles = []
        
        # Check for nearby obstacles (trees, flooded areas, etc.)
        for obstacle in obstacles:
            distance = np.linalg.norm(np.array(self.position) - np.array(obstacle))
            if distance < self.sensing_radius:  # Fixed sensing radius for avoidance, you can adjust this value
                nearby_obstacles.append(obstacle)
        
        # Check for nearby drones (drone positions)
        for drone_pos in other_drone_positions:
            distance = np.linalg.norm(np.array(self.position) - np.array(drone_pos))
            if distance < self.sensing_radius:  # Fixed sensing radius for avoiding other drones
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
    def __init__(self, drone_dict):
        # Initialize PyBullet and load resources
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Load environment elements
        self.load_ground()

        # Parameters
        self.radius = 5
        self.altitude = 2
        self.speed = 100

        self.sensing_radius = 5;

        # Initialize obstacles
        self.trees = self.create_random_trees(20)
        self.flooded_areas = self.create_random_flooded_areas(50)
        self.obstacles = [p.getBasePositionAndOrientation(tree)[0] for tree in self.trees] + \
                         [p.getBasePositionAndOrientation(flood)[0] for flood in self.flooded_areas]
        
        self.drone_dict = drone_dict
        # Initialize drones
        self.drones = self.create_drones(drone_dict)

        # Start the simulation timer
        self.start_time = time.time()

        self.flooded_coordinates = []


    def load_ground(self):
        # Load the transparent plane URDF
        ground_uid = p.loadURDF("plane_transparent.urdf", [0, 0, 0])
        p.changeVisualShape(ground_uid, -1, rgbaColor=[0, 1, 0, 1])  # Solid green color

    def load_drone(self, position):
        # Load the drone model at a specified position
        drone_urdf = "./urdf/quadroter.urdf"
        drone_id = p.loadURDF(drone_urdf, basePosition=position, useFixedBase=False)

        # Short delay after loading the drone
        time.sleep(1)
        return drone_id

    def create_drones(self, drone_dict):
        drone_ids = []
        drones = []
        
        for drone_id, info in self.drone_dict.items():
            # x_offset = random.uniform(-10, 10)
            # y_offset = random.uniform(-10, 10)
            x_offset = info["lats"][0]
            y_offset = info["lons"][0]
            drone_id = self.load_drone([x_offset, y_offset, 1])
            print(drone_id)
            
            radius = random.uniform(3, 6)
            speed = random.uniform(50, 100)
            drone = Drone(drone_id, [x_offset, y_offset, 1], radius, 2, speed, self.sensing_radius)
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
    
    def nearest_color(self, image_array):
        # Calculate the average color of the NXN image
        average_color = np.mean(image_array.reshape(-1, 3), axis=0)

        # Define a set of target colors to match (RGB values)
        target_colors = np.array([
            [0, 255, 0],    # Green
            [0, 0, 50],    # Blue
        ])

        # Use KDTree for nearest neighbor search
        kdtree = KDTree(target_colors)
        _, idx = kdtree.query(average_color)

        nearest_color = target_colors[idx]

        print("Average color of the image:", average_color)
        print("Nearest color match:", nearest_color)
        # Checks if nearest_color is blue
        return nearest_color == [0, 0, 50]


    def capture_aerial_view(self, drone):
        drone_id = drone.getID()
        # Set the folder for saving images
        output_folder = "drone_images"
        os.makedirs(output_folder, exist_ok=True)
        
        # Capture image from an aerial perspective
        width, height = 128, 128
        fov, aspect, near, far = 60, width / height, 0.1, 100
        
        # Get the drone's position and set the camera above it, looking down
        drone_position = p.getBasePositionAndOrientation(drone_id)[0]
        camera_position = [drone_position[0], drone_position[1], drone_position[2] + 10]  # 10 units above the drone
        view_matrix = p.computeViewMatrix(camera_position, drone_position, [0, 1, 0])  # Camera looks directly down
        
        proj_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
        img = p.getCameraImage(width, height, view_matrix, proj_matrix)
        
        # Extract the RGB image data
        img_rgb = np.array(img[2], dtype=np.uint8)
        img_rgb = img_rgb.reshape((height, width, 4))  # Reshape to (H, W, 4)
        
        # Convert from RGBA to BGR for OpenCV
        img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGBA2BGR)
        is_flooded = self.nearest_color(img_bgr)
        print(is_flooded.all())
        if is_flooded.all():
            self.flooded_coordinates.append(drone_position)
            print(f"Flooded area detected at {drone_position}")
        
        # Save the image to a folder
        timestamp = int(time.time())
        filename = os.path.join(output_folder, f"aerial_view_{timestamp}.png")
        cv2.imwrite(filename, img_bgr)
        
        print(f"Aerial view image saved to {filename}")

    def plot_flooded_areas(self):
        # Extract x and y coordinates for flooded areas
        x_coords = [coord[0] for coord in self.flooded_coordinates]
        y_coords = [coord[1] for coord in self.flooded_coordinates]

        # Create a scatter plot of the flood areas on the plane
        plt.figure(figsize=(10, 10))
        plt.scatter(x_coords, y_coords, c='blue', label="Flooded Area")
        plt.xlim(-10, 10)
        plt.ylim(-10, 10)
        
        # Add visual markers for trees
        tree_x = [p.getBasePositionAndOrientation(tree)[0][0] for tree in self.trees]
        tree_y = [p.getBasePositionAndOrientation(tree)[0][1] for tree in self.trees]
        plt.scatter(tree_x, tree_y, c='green', marker='^', label="Trees")

        # Labels and plot details
        plt.title("Detected Flooded Areas on Plane")
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.legend()
        plt.grid(True)

        # Save or display the plot
        plt.savefig("flooded_areas_map.png")
        plt.show()

    def run_simulation(self):
        while True:
            p.stepSimulation()
            current_time = time.time() - self.start_time
            
            i = 0

            # Update position of each drone independently
            for drone in self.drones:
                # Capture drone POV and save the image
                self.capture_aerial_view(drone)
                waypoints = self.drone_dict[str(drone.getID())]
                if i > len(waypoints["lats"]):
                    continue
                t_x = waypoints["lats"][i]
                t_y = waypoints["lons"][i]
                t_z = 5
                target_position = (t_x,t_y,t_z)
                drone.update_position(current_time, self.drones, self.obstacles, target_position)
                
                i+=1
             
            # Delay for simulation timing
            time.sleep(1./240.)

            if current_time > 20:  # Run for 10 seconds
                break
           
        # Plot the flooded areas after the simulation ends
        self.plot_flooded_areas()

# Instantiate and run the drone simulation with 5 drones
if __name__ == "__main__":
    # drone_dict = {'71': {'lats': [29.717102, 29.7170808, 29.7168997, 29.7167764, 29.7164085, 29.7163768, 29.7163433, 29.7155523, 29.7151516, 29.7150883, 29.7151095, 29.7151355, 29.7153262, 29.715385, 29.7162585, 29.7162968, 29.7163315, 29.7164142, 29.7166818, 29.7167244, 29.7167507, 29.7167969, 29.7171419, 29.7172208, 29.7179413, 29.7179845, 29.7180097, 29.7181939, 29.7182927, 29.7183842, 29.7184468, 29.7185099, 29.7187135, 29.7186545, 29.7182801, 29.7180584, 29.7179155, 29.7177239, 29.7174624, 29.717102], 'lons': [-95.4002434, -95.400226, -95.400682, -95.401, -95.4019292, -95.402005, -95.4020917, -95.4040962, -95.4051232, -95.4057174, -95.4058145, -95.4059004, -95.4062185, -95.4061494, -95.4064177, -95.4064012, -95.4063863, -95.406339, -95.4060508, -95.4059569, -95.4058988, -95.4057836, -95.4049237, -95.4047262, -95.4029238, -95.4028156, -95.4027525, -95.402282, -95.4020296, -95.4017839, -95.4016278, -95.4016497, -95.4011212, -95.4010872, -95.4008716, -95.400744, -95.4006371, -95.4005726, -95.4004429, -95.4002434], 'path_length': 1549.3090000000004}, '72': {'lats': [29.717102, 29.7170808, 29.717175, 29.7177629, 29.717797, 29.7177451, 29.7165818, 29.7166015, 29.7166178, 29.716535, 29.7161219, 29.7185801, 29.7185345, 29.7205001, 29.7205597, 29.7203567, 29.7203702, 29.7193633, 29.7190397, 29.7190142, 29.7187883, 29.7187191, 29.7186985, 29.718395, 29.7178241, 29.717797, 29.7177629, 29.717175, 29.7170808, 29.717102], 'lons': [-95.4002434, -95.400226, -95.3999849, -95.3984879, -95.398401, -95.3983733, -95.397752, -95.3977014, -95.3976593, -95.3976174, -95.3974005, -95.3954981, -95.3954275, -95.3938754, -95.3939423, -95.3940902, -95.3941106, -95.3966485, -95.3964867, -95.3965469, -95.3971018, -95.3972772, -95.3973272, -95.397245, -95.3983275, -95.398401, -95.3984879, -95.3999849, -95.400226, -95.4002434], 'path_length': 1821.355}, '73': {'lats': [29.717102, 29.7174624, 29.7177239, 29.7179155, 29.7180584, 29.7182801, 29.7186545, 29.7187135, 29.7190112, 29.7194467, 29.719812, 29.7195767, 29.7192498, 29.7191331, 29.7190218, 29.7186985, 29.718395, 29.7183652, 29.7180452, 29.7180019, 29.7179235, 29.7172863, 29.716535, 29.7161219, 29.7160191, 29.7159441, 29.7143904, 29.714492, 29.7145221, 29.7145642, 29.715184, 29.7158986, 29.7161439, 29.7163249, 29.7170247, 29.7170808, 29.717102], 'lons': [-95.4002434, -95.4004429, -95.4005726, -95.4006371, -95.400744, -95.4008716, -95.4010872, -95.4011212, -95.4003794, -95.3992631, -95.397981, -95.3977866, -95.3976259, -95.3975646, -95.3975106, -95.3973272, -95.397245, -95.3972054, -95.3969652, -95.3968448, -95.3968015, -95.3972172, -95.3976174, -95.3974005, -95.3974801, -95.3975411, -95.3988051, -95.3989217, -95.3988966, -95.398963, -95.399319, -95.3995833, -95.3997126, -95.3998108, -95.400183, -95.400226, -95.4002434], 'path_length': 1620.034}}
    drone_dict = dict()
    for i in range(1,4):
        lats = []
        lons = []
        for j in range(10):
            lats.append(random.uniform(-10, 10))
            lons.append(random.uniform(-10, 10))
        drone_dict[str(70+i)] = {"lats": lats, "lons":lons}

    # API_URL_BASE: str = "http://168.5.37.177/api/v1/"   # wisha's address
    # response = requests.get(API_URL_BASE + "waypoints")
    # response.raise_for_status()
    # drone_dict = response.json()

    print(drone_dict)

    simulation = DroneSimulation(drone_dict)

    simulation.run_simulation()
