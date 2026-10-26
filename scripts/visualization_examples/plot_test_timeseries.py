import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import time

# Create a figure and an axis
fig, ax = plt.subplots()

# Initialize the data
xdata, ydata = [], []
line, = ax.plot([], [], 'r-', label="Real-time Data")
ax.set_xlim(0, 10)  # Set the x-axis range
ax.set_ylim(-1, 1)  # Set the y-axis range
ax.set_xlabel('Time')
ax.set_ylabel('Value')
ax.legend()

# Initialize function to set up the plot
def init():
    line.set_data([], [])
    return line,

# Update function that will be called at each frame
def update(frame):
    # Example: simulate some time series data (sin wave here)
    current_time = time.time()  # You could replace this with actual timestamp data
    xdata.append(current_time)
    ydata.append(np.sin(current_time))  # Use actual data here

    # Update plot
    ax.set_xlim(current_time - 10, current_time)  # Shift the window to the latest 10 seconds of data
    line.set_data(xdata, ydata)
    return line,

# Create the animation
ani = FuncAnimation(fig, update, frames=np.arange(0, 100), init_func=init, blit=True, interval=1)

# Show the plot
plt.show()