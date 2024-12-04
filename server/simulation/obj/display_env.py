import pybullet as p
import pybullet_data
import random
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setGravity(0, 0, -9.8)

# Load OBJ as visual and collision shapes
obj_path = "/Users/admin/Documents/drone-flood-detection/server/simulation/obj/riceu_env.obj"
visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName=obj_path,
    meshScale=[0.1, 0.1, 0.1]
)

collision_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName=obj_path,
    meshScale=[0.1, 0.1, 0.1]
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

def create_water_patches(num_patches=50):
    blue_color = [0, 0, 1, 0.7]
    
    # Much larger area for water patches
    min_x = -50
    max_x = 50
    min_y = -50
    max_y = 50
    
    # Building area to avoid (approximate)
    building_center_x = 1
    building_center_y = 0
    building_radius = 3  # Adjust based on your building size

    for _ in range(num_patches):
        while True:
            # Random size for each patch
            size = random.uniform(0.5, 3.0)  # Made patches potentially larger
            
            # Random position
            x = random.uniform(min_x, max_x)
            y = random.uniform(min_y, max_y)
            
            # Check if the patch is too close to the building
            distance_to_building = np.sqrt((x - building_center_x)**2 + (y - building_center_y)**2)
            if distance_to_building > building_radius:
                break  # Position is okay, exit the while loop
        
        # Create visual shape for water patch
        water_visual = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[size/2, size/2, 0.01],
            rgbaColor=blue_color
        )
        
        # Create water patch
        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=water_visual,
            basePosition=[x, y, 0.01]
        )

# Create more water patches over larger area
create_water_patches(1000)  # Increased number of patches

# Keep simulation running
while True:
    p.stepSimulation()