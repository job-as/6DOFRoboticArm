import pybullet as p
import time

# Connect to PyBullet GUI
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

# Load the robot URDF
robot_id = p.loadURDF("robot.urdf", [0, 0, 0], useFixedBase=True)

# Joint index for "joint1" is 0 (since it's the first joint in URDF)
joint_index = 0

# Move the joint from 0° to 90°
for angle_deg in range(0, 91, 5):
    angle_rad = angle_deg * 3.1416 / 180
    p.setJointMotorControl2(robot_id, joint_index, p.POSITION_CONTROL, targetPosition=angle_rad, force=10)
    p.stepSimulation()
    time.sleep(0.05)

# Hold position for a few seconds
time.sleep(30)
p.disconnect()
