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
import sys


class Drone:
    def __init__(self, drone_id, start_position, radius, altitude, speed, sensing_radius):
        self.drone_id = drone_id
        self.position = start_position
        self.radius = radius
        self.altitude = altitude
        self.speed = speed
        self.center_position = [random.uniform(-10, 10), random.uniform(-10, 10), self.altitude]  # Random center
        self.sensing_radius = sensing_radius
        
        # Create the translucent red sphere to visualize the drone
        self.sphere_id = self.create_red_sphere(start_position)

    def create_red_sphere(self, position):
        # Create a translucent red sphere at the initial drone position
        sphere_radius = self.sensing_radius  # Radius of the sphere
        rgba_color = [1, 0, 0, 0.2]  # Red color with 50% transparency
        sphere_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=rgba_color)
        
        # Create a kinematic multi-body with no mass (i.e., just a visual object)
        sphere_id = p.createMultiBody(
            baseMass=0,  # No mass, making the object kinematic
            baseCollisionShapeIndex=-1,  # No collision for the sphere
            baseVisualShapeIndex=sphere_visual_shape,  # Visual shape is the red sphere
            basePosition=position  # Initial position of the sphere
        )
        
        return sphere_id

    def getID(self):
        return int(self.drone_id)
    
    def getPos(self):
        return self.position

    def update_position(self, current_time, drones, obstacles, target_position, avoid_collisions):
       
        # Gather positions of other drones to avoid collisions
        other_drone_positions = [d.getPos() for d in drones if d.getID() != self.drone_id]
        nearby_obstacles = self.get_nearby_obstacles(obstacles, other_drone_positions)

        # Use motion planning to move towards the target while avoiding obstacles, including other drones
        apply_motion_planning(self.drone_id, target_position, nearby_obstacles, speed=50, avoid_collisions=avoid_collisions)

        # Update the drone's position
        self.position = p.getBasePositionAndOrientation(self.drone_id)[0]

        for o in nearby_obstacles:
            if o == target_position:
                print("********* COLLISION *********")

        
        # Now, update the position of the sphere to match the drone's position
        p.resetBasePositionAndOrientation(self.sphere_id, self.position, [0, 0, 0, 1])  # No rotation
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
                print("obstacle detected!")
        
        # Check for nearby drones (drone positions)
        for drone_pos in other_drone_positions:
            print("drone pos: " + str(drone_pos))
            distance = np.linalg.norm(np.array(self.position) - np.array(drone_pos))
            print("distance = " + str(distance))
            if distance < self.sensing_radius:  # Fixed sensing radius for avoiding other drones
                nearby_obstacles.append(drone_pos)
                print("neighbor drone detected!")

        return nearby_obstacles

    def get_position(self):
        return self.position


class DroneSimulation:
    def __init__(self, waypoints_list, avoid_collisions):
        # Initialize PyBullet and load resources
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Load environment elements
        self.load_ground()

        # Parameters
        self.radius = 5
        self.altitude = 2
        self.speed = 100

        self.avoid_collisions = avoid_collisions

        self.sensing_radius = 2;

        # Initialize obstacles
        self.trees = self.create_random_trees(20)
        self.flooded_areas = self.create_random_flooded_areas(50)
        self.obstacles = [p.getBasePositionAndOrientation(tree)[0] for tree in self.trees] + \
                         [p.getBasePositionAndOrientation(flood)[0] for flood in self.flooded_areas]
        
        self.waypoints_list = waypoints_list

        # Initialize drones
        self.drones, self.drone_ids = self.create_drones()

        self.drone_dict = self.make_drone_dict()

        # Start the simulation timer
        self.start_time = time.time()

        self.flooded_coordinates = []

    def make_drone_dict(self):
        drone_dict = dict()
        i = 0
        for info in self.waypoints_list:
            drone_dict[str(self.drone_ids[i])] = info
            i+=1
        return drone_dict

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

    def create_drones(self):
        drone_ids = []
        drones = []

        for info in self.waypoints_list:
            x_offset = info["lats"][0]
            y_offset = info["lons"][0]
            drone_id = self.load_drone([x_offset, y_offset, 1])
            print(drone_id)
            
            radius = random.uniform(3, 6)
            speed = random.uniform(50, 100)
            drone = Drone(drone_id, [x_offset, y_offset, 1], radius, 2, speed, self.sensing_radius)
            drones.append(drone)
            drone_ids.append(drone_id)

        # print(drones)
        return drones, drone_ids

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

        # print("Average color of the image:", average_color)
        # print("Nearest color match:", nearest_color)
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
        
        # print(f"Aerial view image saved to {filename}")

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
        path = []
        i = 0
        while True:
            p.stepSimulation()
            current_time = time.time() - self.start_time
            print(self.drones)
            # Update position of each drone independently
            for drone in self.drones:
                # Capture drone POV and save the image
                self.capture_aerial_view(drone)
                id = str(drone.getID())
                waypoints = self.drone_dict[id]
                if i >= len(waypoints["lats"]):
                    continue
                t_x = waypoints["lats"][i]
                t_y = waypoints["lons"][i]
                t_z = 0.8
                target_position = (t_x,t_y,t_z)
                drone.update_position(current_time, self.drones, self.obstacles, target_position, self.avoid_collisions)
                if (drone.getID()) == 71:
                    path.append(drone.position)
                
            i+=1
            # Delay for simulation timing
            time.sleep(1./240.)

            if current_time > 20:  # Run for 10 seconds
                break
           
        # Plot the flooded areas after the simulation ends
        self.plot_flooded_areas()
        # print(path)
        

def collision_demo(avoid_collision=True):
    waypoints_list = [{"lats": [i for i in range(-5,5)], "lons": [i for i in range(-5,5)]},
     {"lats": [i for i in range(-5,5)], "lons": [i for i in range(5,-5,-1)]},
     {"lats": [i for i in range(5,-5,-1)], "lons": [i for i in range(5,-5,-1)]}]
    simulation = DroneSimulation(waypoints_list, avoid_collision)
    simulation.run_simulation()

# Instantiate and run the drone simulation with 5 drones
if __name__ == "__main__":
    collision_demo(True)



    # for i in range(1,num_drones-1):
    #     lats = []
    #     lons = []
    #     for j in range(10):
    #         lats.append(random.uniform(-10, 10))
    #         lons.append(random.uniform(-10, 10))
    #     drone_dict[str(70+i)] = {"lats": lats, "lons":lons}

    



    # API_URL_BASE: str = "http://168.5.37.177/api/v1/"   # wisha's address
    # response = requests.get(API_URL_BASE + "waypoints")
    # response.raise_for_status()
    # drone_dict = response.json()

    # print(drone_dict)

    # simulation = DroneSimulation(drone_dict)

    # simulation.run_simulation()
    