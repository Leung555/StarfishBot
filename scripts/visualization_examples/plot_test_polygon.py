import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from matplotlib.animation import FuncAnimation

# Optional: Use Agg backend for non-interactive use (fix for some environments)
import matplotlib
matplotlib.use('Agg')

# Step 1: Create a polygon
# Define the coordinates of the polygon (e.g., a square)
coordinates = [(1, 1), (5, 1), (5, 5), (1, 5)]
polygon = Polygon(coordinates)

# Step 2: Calculate the Center of Gravity (COG)
cog = polygon.centroid

# Step 3: Set up the plot
fig, ax = plt.subplots()
ax.set_xlim(0, 6)
ax.set_ylim(0, 6)

# Step 4: Initialize the plot elements (polygon and COG)
polygon_patch = plt.Polygon(coordinates, closed=True, edgecolor='black', facecolor='lightblue', alpha=0.5)
ax.add_patch(polygon_patch)
cog_point, = ax.plot([], [], 'ro')  # COG will be plotted as a red point

# Function to update the plot in real-time
def update(frame):
    # You can change the polygon coordinates here to simulate movement (e.g., animate the polygon)
    new_coordinates = [(x + np.sin(frame * 0.1), y + np.cos(frame * 0.1)) for x, y in coordinates]
    new_polygon = Polygon(new_coordinates)
    
    # Update the polygon shape
    polygon_patch.set_xy(new_coordinates)
    
    # Update the COG
    new_cog = new_polygon.centroid
    cog_point.set_data(new_cog.x, new_cog.y)
    
    return polygon_patch, cog_point

# Step 5: Animate the plot in real-time
ani = FuncAnimation(fig, update, frames=np.arange(0, 100), interval=100, blit=True)

# Show the plot
plt.show()
