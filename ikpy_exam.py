from ikpy.chain import Chain
import numpy as np
import matplotlib.pyplot as plt
from math import pi

# Load the URDF file with proper active links mask
robot_chain = Chain.from_urdf_file(
    "6dof_robot.urdf",
    active_links_mask=[False, True, True, True, True, True, True]  # First link is fixed base
)

# Define target position [x, y, z]
target_position = [0.4, 0.1, 0.4]

# Define orientation as identity matrix (default)
target_orientation = np.eye(3)

# Solve IK
ik_solution = robot_chain.inverse_kinematics(
    target_position=target_position,
    target_orientation=target_orientation,
    orientation_mode="all"
)

# Print results (skip the first element which corresponds to the fixed base)
print("Joint angles (radians):", ik_solution[1:])
print("Joint angles (degrees):", np.degrees(ik_solution[1:]))

# Create a matplotlib figure and axis
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')

# Plot the robot
robot_chain.plot(
    ik_solution,
    ax=ax,  # Pass the axis object
    target=target_position
)

# Set plot limits and labels
ax.set_xlim(-0.5, 0.5)
ax.set_ylim(-0.5, 0.5)
ax.set_zlim(0, 1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.title('Robot Arm IK Solution')
plt.show()