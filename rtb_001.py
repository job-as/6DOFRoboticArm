import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib.animation import FuncAnimation

# Define start and end points
start = np.array([0, 0, 0])
end = np.array([5, 3, 8])

# Number of steps for animation
steps = 50
x_vals = np.linspace(start[0], end[0], steps)
y_vals = np.linspace(start[1], end[1], steps)
z_vals = np.linspace(start[2], end[2], steps)

# Set up 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.set_box_aspect([1, 1, 1])
ax.set_xlim(0, 6)
ax.set_ylim(0, 4)
ax.set_zlim(0, 9)

# Plot empty line and moving point
line, = ax.plot([], [], [], color='red')
point, = ax.plot([], [], [], 'bo')

# Initialization function
def init():
    line.set_data([], [])
    line.set_3d_properties([])
    point.set_data([], [])
    point.set_3d_properties([])
    return line, point

# Update function for each frame
def update(frame):
    line.set_data(x_vals[:frame], y_vals[:frame])
    line.set_3d_properties(z_vals[:frame])
    point.set_data([x_vals[frame-1]], [y_vals[frame-1]])
    point.set_3d_properties([z_vals[frame-1]])
    return line, point

# Create animation
ani = FuncAnimation(fig, update, frames=steps, init_func=init, interval=100, blit=True)

plt.show()
