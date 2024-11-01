import numpy as np
from scipy.spatial import KDTree
from PIL import Image

def nearest_color(img_path = ""):
    image = Image.open(img_path).convert("RGB")
    image_array = np.array(image)

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
    print(idx)

    nearest_color = target_colors[idx]

    print("Average color of the image:", average_color)
    print("Nearest color match:", nearest_color)
    # Checks if nearest_color is blue
    return nearest_color == [0, 0, 50]


print(nearest_color("/Users/andrewondara/drone-flood-detection/server/simulation/drone_images/aerial_view_1730416454.png"))

    