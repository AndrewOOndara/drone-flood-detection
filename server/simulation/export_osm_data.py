import osmnx as ox
import geopandas as gpd
import matplotlib.pyplot as plt

# Specify the area you are interested in (Rice University)
place_name = "Rice University, Houston, Texas, USA"

# Fetch the graph of streets in the area (network of roads)
graph = ox.graph_from_place(place_name, network_type='all')

# Fetch building footprints in the area
gdf_buildings = ox.geometries_from_place(place_name, tags={'building': True})

# Plot the street network and buildings
fig, ax = plt.subplots(figsize=(10, 10))

# Plot buildings
gdf_buildings.plot(ax=ax, facecolor='lightgray', edgecolor='black', alpha=0.7)

# Plot the street network on top of buildings
ox.plot_graph(ox.project_graph(graph), ax=ax, node_size=0, edge_color='black', edge_linewidth=0.5, show=False)

# Display the plot
plt.show()
