import numpy as np
import pybullet as p
import pybullet_data
import time
import random
import math
import cv2  
import os 
# import osmnx
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
        p.setCollisionFilterGroupMask(sphere_id, -1, 0, 0)
        
        return sphere_id

    def getID(self):
        return int(self.drone_id)
    
    def getPos(self):
        return self.position

    def update_position(self, current_time, drones, obstacles, target_position, avoid_collisions,initiate_landing):
       
        # Gather positions of other drones to avoid collisions
        other_drone_positions = [d.getPos() for d in drones if d.getID() != self.drone_id]
        nearby_obstacles = self.get_nearby_obstacles(obstacles, other_drone_positions)

        # Use motion planning to move towards the target while avoiding obstacles, including other drones
        apply_motion_planning(self.drone_id, target_position, nearby_obstacles, self.speed, avoid_collisions=avoid_collisions, initiate_landing=initiate_landing)

        # Update the drone's position
        self.position = p.getBasePositionAndOrientation(self.drone_id)[0]
        # Now, update the position of the sphere to match the drone's position
        p.resetBasePositionAndOrientation(self.sphere_id, self.position, [0, 0, 0, 1])  # No rotation
        pos,ori = p.getBasePositionAndOrientation(self.sphere_id)
        # if p.getBasePositionAndOrientation(self.sphere_id)[0] != self.position:
        #     print("SPHERE IS BEING BADDDDD")

        # for o in nearby_obstacles:
        #     if o == target_position:
        #         print("********* COLLISION *********")


        #print(f"Drone {self.drone_id} position: {self.position}")
        # print(f"Sphere {self.sphere_id} position: {pos}")


    def get_nearby_obstacles(self, obstacles, other_drone_positions):
        # This function checks which obstacles are within the sensing radius of the drone
        nearby_obstacles = []
        
        # Check for nearby obstacles (trees, flooded areas, etc.)
        for obstacle in obstacles:
            distance = np.linalg.norm(np.array(self.position) - np.array(obstacle))
            if distance < self.sensing_radius:  # Fixed sensing radius for avoidance, you can adjust this value
                nearby_obstacles.append(obstacle)
                #print("obstacle detected!")
        
        # Check for nearby drones (drone positions)
        for drone_pos in other_drone_positions:
            #print("drone pos: " + str(drone_pos))
            distance = np.linalg.norm(np.array(self.position) - np.array(drone_pos))
            #print("distance = " + str(distance))
            if distance < self.sensing_radius:  # Fixed sensing radius for avoiding other drones
                nearby_obstacles.append(drone_pos)
                #print("neighbor drone detected!")

        return nearby_obstacles

    def get_position(self):
        return self.position


class DroneSimulation:
    def __init__(self, waypoints_list, avoid_collisions):
        # Initialize PyBullet and load resources
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        
        # Load environment elements
        self.ground_uid = self.load_ground()

        # Parameters
        self.radius = 5
        self.altitude = 2
        self.speed = 50
        self.sensing_radius = 2

        self.avoid_collisions = avoid_collisions

        # Initialize obstacles
        self.trees = self.create_random_trees(2)
        # self.trees = []
        self.flooded_areas = self.create_random_flooded_areas(40)
        self.obstacles = [p.getBasePositionAndOrientation(tree)[0] for tree in self.trees] + \
                         [p.getBasePositionAndOrientation(flood)[0] for flood in self.flooded_areas] + \
                         [p.getBasePositionAndOrientation(self.ground_uid)[0]]
        
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
        #print(drone_dict)
        return drone_dict
    

    # Function to invert the transformation
    def inverse_transform_coordinates(self, x, y, latO = 29.7172, lonO = -95.4043, rot = -1.50002, scale = 1027.52):
        print(f"THIS IS X {x}")
        print(f"THIS IS Y {y}")
        # Inverse of the scale factor
        inv_scale = 1 / scale
        
        # Inverse rotation matrix (transpose of the original rotation matrix)
        inverse_rotation_matrix = np.array([
            [np.cos(rot), -np.sin(rot)],
            [np.sin(rot), np.cos(rot)]
        ])
        
        # Apply inverse rotation and scale
        transformed_coords = np.array([x, y]) * inv_scale  # Reverse scaling
        original_coords = inverse_rotation_matrix @ transformed_coords  # Apply inverse rotation
        
        # Add back the latitude and longitude offsets
        lat = original_coords[0] + latO
        lon = original_coords[1] + lonO
        
        return lat, lon
    
    def converted_flooded_coordinates(self, flooded_coordinates):
        """
        Converts a list of Cartesian coordinates into latitude and longitude tuples.
        
        Args:
            flooded_coordinates (list of tuple): List of (x, y) coordinates to convert.
        
        Returns:
            list of tuple: Converted coordinates in (latitude, longitude) format.
        """
        lat_lon_coordinates = []
        for flood in flooded_coordinates:
            print(f"THIS IS A FLOOD {type(flood)}")
            print(f"{flood[0]}{flood[1]}{flood[2]}")
            lat, lon = self.inverse_transform_coordinates(flood[0], flood[1])
            lat_lon_coordinates.append((lat, lon))
        return lat_lon_coordinates

    def load_ground(self):
        # Load the transparent plane URDF
        ground_uid = p.loadURDF("plane_transparent.urdf", [0,0,0.005])
        #p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        p.changeVisualShape(ground_uid, -1, rgbaColor=[0,1,0,1])
        # Load OBJ as visual and collision shapes
        obj_path = "/Users/admin/Documents/drone-flood-detection/server/simulation/obj/riceu_env.obj"
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=obj_path,
            meshScale=[0.01, 0.01, 0.01]
        )

        collision_shape_id = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=obj_path,
            meshScale=[0.01, 0.01, 0.01]
        )
        
        # Rotate the object by 90 degrees around the X-axis
        orientation = p.getQuaternionFromEuler([1.5708, 0, 0])
        
        # Create the building
        building = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=[1, 0, 0],
            baseOrientation=orientation
        )
        
        return building

    def load_drone(self, position):
        # Load the drone model at a specified position
        drone_urdf = "./urdf/quadroter.urdf"
        drone_id = p.loadURDF(drone_urdf, basePosition=position, useFixedBase=False)
        p.setCollisionFilterGroupMask(drone_id, -1, collisionFilterGroup=1, collisionFilterMask=0)

        # Short delay after loading the drone
        # time.sleep(1)
        return drone_id

    def create_drones(self):
        drone_ids = []
        drones = []

        for info in self.waypoints_list:
            x_offset = info["lats"][0]
            y_offset = info["lons"][0]
            drone_id = self.load_drone([x_offset, y_offset, 1])
            #print(drone_id)
            
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
            x = 4
            y = 2
            z = 0.5
            tree_urdf = "./urdf/tree.urdf"
            tree_id = p.loadURDF(tree_urdf, basePosition=[x, y, z])
            p.setCollisionFilterGroupMask(tree_id, -1, collisionFilterGroup=2, collisionFilterMask=0)
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
        if 1 == 1:
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

    def plot_drone_paths(self, real_paths):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Set up the colormap
        cmap = plt.get_cmap('viridis')  # You can change the colormap here
        norm = plt.Normalize()  # Normalize the time or index values

        # Iterate through each path in real_paths
        for id in real_paths:
            lats = real_paths[id]["lats"]
            lons = real_paths[id]["lons"]
            depth = real_paths[id]["depth"]

            # Generate a time-like variable (e.g., index) for coloring the line
            time = np.arange(len(lons))  # Use the index of the points as a proxy for time
            
            # Normalize the time values for color mapping
            norm_time = plt.Normalize(time.min(), time.max())

            # Plot each segment of the line with a color based on time
            for i in range(len(lons) - 1):
                x = [lons[i], lons[i + 1]]
                y = [lats[i], lats[i + 1]]
                z = [depth[i], depth[i + 1]]
                
                # Normalize time for each segment
                color = cmap(norm_time(time[i]))  # Color based on the time of the point
                
                # Plot the segment with the corresponding color
                ax.plot(x, y, z, color=color, lw=2)

        # Adding a colorbar for the colormap
        mappable = plt.cm.ScalarMappable(cmap=cmap, norm=norm_time)
        mappable.set_array(time)
        cbar = fig.colorbar(mappable, ax=ax, shrink=0.5, aspect=5)
        cbar.set_label('Time')

        # Set axis labels
        ax.set_xlabel('Lats')
        ax.set_ylabel('Lons')
        ax.set_zlabel('Depth')

        # Show the plot
        plt.show()
    def convert_floods(self, floods):
        # Convert np.float64 to standard float and return a list of tuples
        return [(float(lat), float(lon)) for lat, lon in floods]

    def send_flood_data(self,converted_floods):
        # Prepare the flooded data for sending
        flooded = converted_floods
        payload = {'flooded_coordinates': flooded}
        
        # Set the URL for the API endpoint
        url = "http://168.5.58.43:5000/api/v1/converted_flood_points"  # Replace with your actual API endpoint
        
        # Set headers for the request
        headers = {'Content-Type': 'application/json'}
        
        # Send the POST request
        response = requests.post(url, data=json.dumps(payload), headers=headers)
        
        # Check the response
        if response.status_code == 200:
            print("Request successful")
        else:
            print(f"Request failed with status code {response.status_code}")

    def run_simulation(self):
        real_paths = dict()
        i_bank = dict()
        for drone in self.drones:
            real_paths[str(drone.getID())] = {"lats":[], "lons":[], "depth":[]}
            i_bank[str(drone.getID())] = 1
        
        while True:
            p.stepSimulation()
            current_time = time.time() - self.start_time

            # Update position of each drone independently
            for drone in self.drones:
                # Capture drone POV and save the image
                self.capture_aerial_view(drone)
                id = str(drone.getID())
                waypoints = self.drone_dict[id]
                thisI = i_bank[id]
                if thisI < len(waypoints["lats"]):
                    
                    drone_pos = drone.position
                    
                    t_x = waypoints["lats"][thisI]
                    t_y = waypoints["lons"][thisI]
                    t_z = 1
                    current_destination_position = np.array([t_x,t_y,t_z])

                    distance_to_waypoint = np.linalg.norm(drone_pos - current_destination_position)
                    
                    DISTANCE_TOLERANCE = 2 # Distance tolerance if needed, set to 0 for strict equality
                    # only give the next waypoint 
                    if distance_to_waypoint <= DISTANCE_TOLERANCE and thisI+1 < len(waypoints["lats"]):
                        #print("giving next target point")
                        t_x_prime = waypoints["lats"][thisI+1]
                        t_y_prime = waypoints["lats"][thisI+1]
                        t_z_prime = 1
                        target_position = np.array([t_x_prime,t_y_prime,t_z_prime])
                        drone.update_position(current_time, self.drones, self.obstacles, target_position, self.avoid_collisions, False)
                        i_bank[id] += 1

                    else:
                        print("Continuing motion to waypoint")
                        drone.update_position(current_time, self.drones, self.obstacles, current_destination_position, self.avoid_collisions, False)

                    
                    # keep track of real drone position
                    r_x = drone_pos[0]
                    r_y = drone_pos[1]
                    r_z = drone_pos[2]
                    real_paths[id]["lats"].append(r_x)
                    real_paths[id]["lons"].append(r_y)
                    real_paths[id]["depth"].append(r_z)
                # elif thisI ==  len(waypoints["lats"]):
                #     print("doing landing for drone " + drone.getID())
                #     drone.update_position(current_time, self.drones, self.obstacles, target_position, self.avoid_collisions, True)

            # Delay for simulation timing
            time.sleep(1/240)

            if current_time > 10:  
                break
           
        # Plot the flooded areas after the simulation ends
        flooded = self.converted_flooded_coordinates(self.flooded_coordinates)
        converted_floods = self.convert_floods(flooded)

        self.send_flood_data(converted_floods)
        self.plot_flooded_areas()
        self.plot_drone_paths(real_paths)
        # print(real_paths)
        
def latlon_2_sim(lat,lon):
        
    ### UNTOUCHABLE CONSTANTS ###
    latO = 29.7172
    lonO = -95.4043
    rot = -1.50002
    scale = 1027.52
    #############################

    rot_matrix = np.array([[np.cos(rot),np.sin(rot)],
               [-np.sin(rot),np.cos(rot)]])
    
    vec = np.array([lat-latO, lon-lonO])

    return np.dot(rot_matrix, vec*scale)


def rice_demo(avoid_collision=True):
    waypoints_list = [{"lats": [i for i in range(-5,5)], "lons": [i for i in range(-5,5)]},
     {"lats": [i for i in range(-5,5)], "lons": [i for i in range(5,-5,-1)]},
     {"lats": [i for i in range(5,-5,-1)], "lons": [i for i in range(5,-5,-1)]}]
    simulation = DroneSimulation(waypoints_list, avoid_collision)
    simulation.run_simulation()

def real_demo(api_url, avoid_collisions=True):
    print("doing real demo")
    API_URL_BASE: str = api_url   # wisha's address
    response = requests.get(API_URL_BASE + "waypoints")
    print(response)
    response.raise_for_status()
    pre_waypoints_list = list(response.json().values())
    print(pre_waypoints_list)
    waypoints_list = []

    for latlondict in pre_waypoints_list:
        new_latlon_dict = {"lats":[], "lons":[]}
        for i in range(len(latlondict["lats"])):
            lat = latlondict["lats"][i]
            lon = latlondict["lons"][i]
            cart = list(latlon_2_sim(lat,lon))
            new_latlon_dict["lats"].append(cart[0])
            new_latlon_dict["lons"].append(cart[1])
        waypoints_list.append(new_latlon_dict)

    print("after transform: " + str(waypoints_list))

    simulation = DroneSimulation(waypoints_list, avoid_collisions)
    simulation.run_simulation()

# Instantiate and run the drone simulation with 5 drones
if __name__ == "__main__":

    avoid_collisions = True
    rice_demo(avoid_collisions)

    api_url = "http://168.5.58.43:5000/api/v1/"
    # real_demo(api_url, avoid_collisions)



    # for i in range(1,num_drones-1):
    #     lats = []
    #     lons = []
    #     for j in range(10):
    #         lats.append(random.uniform(-10, 10))
    #         lons.append(random.uniform(-10, 10))
    #     drone_dict[str(70+i)] = {"lats": lats, "lons":lons}

    # print(drone_dict)

    # simulation = DroneSimulation(drone_dict)

    # simulation.run_simulation()
    