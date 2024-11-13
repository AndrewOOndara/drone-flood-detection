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
from rrt import apply_motion_planning 
import matplotlib.pyplot as plt

class DroneSimulation:
    def __init__(self):
        # Initialize PyBullet and load resources
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load environment elements
        self.load_ground()
        self.drone_id = self.load_drone()
        
        # Parameters
        self.radius = 5
        self.altitude = 2
        self.speed = 10
        self.orbit_speed = 10

        # Initialize obstacles
        self.trees = self.create_random_trees(20)
        self.flooded_areas = self.create_random_flooded_areas(100)
        self.obstacles = [p.getBasePositionAndOrientation(tree)[0] for tree in self.trees] + \
                         [p.getBasePositionAndOrientation(flood)[0] for flood in self.flooded_areas]

        # Start the simulation timer
        self.start_time = time.time()

        # Create directory for saving images
        self.image_dir = "drone_images"
        os.makedirs(self.image_dir, exist_ok=True)
        self.flooded_coordinates = []


    def load_ground(self):
        ground_uid = p.loadURDF("plane_transparent.urdf", [0, 0, 0])
        p.changeVisualShape(ground_uid, -1, rgbaColor=[0, 1, 0, 1])  # Solid green color

    def load_drone(self):
        drone_urdf = "/Users/andrewondara/drone-flood-detection/server/simulation/urdf/quadroter.urdf"
        drone_id = p.loadURDF(drone_urdf, basePosition=[5, 0, 1], useFixedBase=False)
        if drone_id < 0:
            raise ValueError("Failed to load the drone model. Check the URDF path and file integrity.")
        time.sleep(1)
        return drone_id

    def create_random_trees(self, num_trees):
        tree_ids = []
        for _ in range(num_trees):
            x, y = random.uniform(-10, 10), random.uniform(-10, 10)
            z = 0.5
            tree_urdf = "/Users/andrewondara/drone-flood-detection/server/simulation/urdf/tree.urdf"
            tree_id = p.loadURDF(tree_urdf, basePosition=[x, y, z])
            tree_ids.append(tree_id)
        return tree_ids

    def create_random_flooded_areas(self, num_areas):
        flood_ids = []
        for _ in range(num_areas):
            x, y = random.uniform(-10, 10), random.uniform(-10, 10)
            length, width = random.uniform(1, 3), random.uniform(1, 3)
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


    def capture_aerial_view(self):
        # Set the folder for saving images
        output_folder = "drone_images"
        os.makedirs(output_folder, exist_ok=True)
        
        # Capture image from an aerial perspective
        width, height = 128, 128
        fov, aspect, near, far = 60, width / height, 0.1, 100
        
        # Get the drone's position and set the camera above it, looking down
        drone_position = p.getBasePositionAndOrientation(self.drone_id)[0]
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

            # Update orbit center position
            center_x = math.cos(self.orbit_speed * current_time) * 3
            center_y = math.sin(self.orbit_speed * current_time) * 3
            center = [center_x, center_y, self.altitude]

            # Circular path for the drone
            target_x = center[0] + self.radius * math.cos(self.speed * current_time)
            target_y = center[1] + self.radius * math.sin(self.speed * current_time)
            target_z = self.altitude
            target_position = [target_x, target_y, target_z]

            # Motion planning with obstacle avoidance
            apply_motion_planning(self.drone_id, target_position, self.obstacles, speed=50)

            # Capture drone POV and save the image
            self.capture_aerial_view()
            time.sleep(1./240.)
            # End condition for the simulation (e.g., run for 10 seconds)
            if current_time > 20:  # Run for 10 seconds
                break

        # Plot the flooded areas after the simulation ends
        self.plot_flooded_areas()

# Run the drone simulation
if __name__ == "__main__":
    simulation = DroneSimulation()
    simulation.run_simulation()
