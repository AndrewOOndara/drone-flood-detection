import numpy as np
import pybullet as p
import math

def calculate_avoidance_vector(drone_pos, obstacles, avoidance_radius=5):
    """
    Calculate a vector to avoid obstacles near the drone.

    :param drone_pos: The current position of the drone.
    :param obstacles: List of obstacle positions (e.g., trees, flooded areas).
    :param avoidance_radius: The distance threshold for avoiding obstacles.
    :return: A numpy array with the avoidance vector.
    """
    avoidance_vector = np.array([0.0, 0.0, 0.0])

    for obs_pos in obstacles:
        distance = np.linalg.norm(np.array(drone_pos) - np.array(obs_pos))/2
        
        if distance < avoidance_radius:
            # Calculate vector away from the obstacle
            away_vector = np.array(drone_pos) - np.array(obs_pos)
            # print("away vector = " + str(away_vector))
            # print("distance = " + str(distance))
            
            # print("computation 1: " + str(away_vector/distance))
            # print("computation 2: " + str(avoidance_radius-distance))
            scaled_away_vector = (away_vector / distance) * (avoidance_radius - distance)
            avoidance_vector = np.add(avoidance_vector, scaled_away_vector)
            # print("avoiding obstacle with avoidance vector " + str(avoidance_vector))
            # print("old vector = " + str(scaled_away_vector) + ", new vector = " + str(avoidance_vector))
    
    return avoidance_vector

def apply_motion_planning(drone_id, target_pos, obstacles, speed=50, avoid_collisions=True):
    """
    Apply forces to the drone to move toward a target position while avoiding obstacles.

    :param drone_id: The ID of the drone in the PyBullet simulation.
    :param target_pos: The target position to move towards.
    :param obstacles: List of positions of obstacles to avoid.
    :param speed: Speed factor for the drone's movement.
    """
    # Get the current position of the drone
    drone_pos, _ = p.getBasePositionAndOrientation(drone_id)
    print(drone_pos)
    print(target_pos)

    # Calculate a movement vector towards the target
    movement_vector = np.array(target_pos) - np.array(drone_pos)
    if (movement_vector != np.zeros(3)).all():
        movement_vector = movement_vector / np.linalg.norm(movement_vector) * speed

    # Calculate avoidance vector based on nearby obstacles
    if avoid_collisions:
        avoidance_vector = calculate_avoidance_vector(drone_pos, obstacles)*10
        print("movement vector = " + str(movement_vector))
    else:
        avoidance_vector = np.zeros(3)
        print("not avoiding anything")

    # Combine movement and avoidance vectors
    combined_vector = movement_vector + avoidance_vector
    force = combined_vector * 100.0  # Scaling factor to apply force

    # Apply the calculated force to the drone
    p.applyExternalForce(drone_id, -1, forceObj=force, posObj=drone_pos, flags=p.WORLD_FRAME)

