# import sympy as sp
# import numpy as np
# x1, x2, x3, x4, x5, x6 = sp.symbols('x1 x2 x3 x4 x5 x6')
# l1, l2, l6 = sp.symbols('l1 l2 l6')
# a_1 = sp.Matrix([
# 		[sp.cos(x1), 0, -sp.sin(x1), 0],
# 		[sp.sin(x1), 0, sp.cos(x1), 0],
# 		[0, -1, 0, l1],
# 		[0, 0, 0, 1],
# 	])

# a_2 = sp.Matrix([
# 		[sp.cos(x2), -sp.sin(x2), 0, l2*sp.cos(x2)],
# 		[sp.sin(x2), sp.cos(x2), 0, l2*sp.cos(x2)],
# 		[0, 0, 1, 0],
# 		[0, 0, 0, 1],
# 	])

# a_3 = sp.Matrix([
# 		[sp.cos(x3), 0, -sp.sin(x3), 0],
# 		[sp.sin(x3), 0, sp.cos(x3), 0],
# 		[0, -1, 0, 0],
# 		[0, 0, 0, 1],
# 	])

# a_4 = sp.Matrix([
# 		[sp.cos(x4), 0, -sp.sin(x4), 0],
# 		[sp.sin(x4), 0, sp.cos(x4), 0],
# 		[0, -1, 0, 0],
# 		[0, 0, 0, 1],
# 	])

# a_5 = sp.Matrix([
# 		[sp.cos(x5), 0, sp.sin(x5), 0],
# 		[sp.sin(x5), 0, -sp.cos(x5), 0],
# 		[0, 1, 0, 0],
# 		[0, 0, 0, 1],
# 	])

# a_6 = sp.Matrix([
# 		[sp.cos(x6), -sp.sin(x6), 0, 0],
# 		[sp.sin(x6), sp.cos(x6), 0, 0],
# 		[0, 0, 1, l6],
# 		[0, 0, 0, 1],
# 	])


# t01 = a_1
# t02 = t01*a_2
# t03 = t02*a_3
# t04 = t03*a_4
# t05 = t04*a_5
# t06 = t05*a_6

# ry = sp.Matrix([
# 		[sp.cos(x1),0,sp.sin(x1)],
# 		[0,1,0],
# 		[-sp.sin(x1),0,sp.cos(x1)]
# 	])
# rz = sp.Matrix([
# 		[sp.cos(x1),-sp.sin(x1),0],
# 		[sp.sin(x1),sp.cos(x1),0],
# 		[0,0,1],
# 	])
# print(sp.simplify(ry*rz*ry))

from ikpy.chain import Chain
from ikpy.link import URDFLink
import numpy as np

# Define robot chain with proper URDFLink parameters
my_chain = Chain(name='6dof_arm', links=[
    URDFLink(
        name="base",
        bounds=[-np.pi, np.pi],
        origin_translation=[0, 0, 0],  # Required in newer versions
        origin_orientation=[0, 0, 0],   # Required in newer versions
        rotation=[0, 0, 1]             # Rotation axis
    ),
    URDFLink(
        name="shoulder",
        bounds=[-np.pi/2, np.pi/2],
        origin_translation=[0, 0, 0.1],  # Link length offset
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0]               # Different rotation axis
    ),
    URDFLink(
        name="elbow",
        bounds=[-np.pi, np.pi],
        origin_translation=[0.5, 0, 0],  # Link length
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0]
    ),
    URDFLink(
        name="wrist1",
        bounds=[-np.pi, np.pi],
        origin_translation=[0.5, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[1, 0, 0]
    ),
    URDFLink(
        name="wrist2",
        bounds=[-np.pi/2, np.pi/2],
        origin_translation=[0, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0]
    ),
    URDFLink(
        name="gripper",
        bounds=[-np.pi, np.pi],
        origin_translation=[0.1, 0, 0],  # End effector offset
        origin_orientation=[0, 0, 0],
        rotation=[1, 0, 0]
    )
])

# Define target position [x, y, z]
target_position = [0.5, 0.5, 0.5]

# Solve IK
ik_solution = my_chain.inverse_kinematics(target_position)
print("Joint angles (radians):", ik_solution)
print("Joint angles (degrees):", np.degrees(ik_solution))