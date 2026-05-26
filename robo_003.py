import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 

## DH-parameters to homologous transformation matrix
def dh_to_htm(parameters):
	a, alpha, d, theta = parameters
	alpha = np.deg2rad(alpha)
	theta = np.deg2rad(theta)
	## short notations
	c_a = np.cos(alpha); s_a = np.sin(alpha)
	c_t = np.cos(theta); s_t = np.sin(theta)
	## homogeneous transformation matrix
	t = np.array([
			[c_t, -s_t*c_a, s_t*s_a, a*c_t],
			[s_t, c_t*c_a, -c_t*s_a, a*s_t],
			[0, s_a, c_a, d],
			[0, 0, 0, 1],
		])
	return t.round(3)
## DH-parameters generate for 6-DOF
def dh_table(thetas):
	## joint angles
	q1,q2,q3,q4,q5,q6 = thetas
	## link length
	l1,l2,l3,l4,l5,l6 = 1,1,1,1,1,1
	## dis-structuring
	q1,q2,q3,q4,q5,q6 = thetas
	## link lengths
	a2,a3,a4 = l2,l3,l4
	## link offset
	d1, d4, d6 = l1, l3+l4, l6

	dh_para = np.array([
			[0, -90, d1, q1],
			[a2, 0, 0, q2-90],
			[0, -90, 0, q3],
			[0, 90, d4, q4],
			[0, -90, 0, q5],
			[0, 0, d6, q6+90],
		])
	return dh_para

## end-effector orientation
def ef_orientation(orn):
	phi, theta, psi = orn
	## degree to rad conversion
	phi = np.deg2rad(phi)
	theta = np.deg2rad(theta)
	psi = np.deg2rad(psi)
	## short notations
	c_ph = np.cos(phi); s_ph = np.sin(phi)
	c_t = np.cos(theta); s_t = np.sin(theta)
	c_ps = np.cos(psi); s_ps = np.sin(psi)
	## rotational matrix for Euler angles ZYZ
	R = np.array([
			[c_ph*c_t*c_ps-s_ph*s_ps, -c_ph*c_t*s_ps-s_ph*c_ps, c_ph*s_t],
			[s_ph*c_t*c_ps+c_ph*s_ps, -s_ph*c_t*s_ps+c_ph*c_ps, s_ph*s_t],
			[-s_t*c_ps, s_t*s_ps, c_t]
		])

	return R.round(3)

### Euler angles calculation for ZYZ configuration
def euler_angle_zyz(T):
	## angles
	angles = []
	## extracting rotation matrix from T
	if T.shape[0] == 4:
		R = T[:-1,:-1]
	else:
		R = T

	## if r_33 = +1
	if R[2,2] == 1:
		theta = np.rad2deg(0)
		## sum of phi and psi can be calculated
		phi = np.rad2deg(0)  #let phi = 0
		psi = np.rad2deg(np.arctan2(R[1,0],R[0,0]))
		angles.append([float(phi), float(theta), float(psi)])
	## if r_33 = -1
	elif R[2,2] == -1:
		theta = np.rad2deg(0)
		## sum of phi and psi can be calculated
		phi = np.rad2deg(0)  #let phi = 0
		psi = np.rad2deg(np.arctan2(-R[0,1],-R[0,0]))
		angles.append([float(phi), float(theta), float(psi)])
	else:
		## first possible solution
		theta = np.rad2deg(np.arctan2(np.sqrt(1-R[2,2]**2), R[2,2]))
		phi = np.rad2deg(np.arctan2(R[1,2], R[0,2]))
		psi = np.rad2deg(np.arctan2(R[2,1], -R[2,0]))
		angles.append([float(phi), float(theta), float(psi)])

		## second possible solution
		theta = np.rad2deg(np.arctan2(-np.sqrt(1-R[2,2]**2), R[2,2]))
		phi = np.rad2deg(np.arctan2(-R[1,2], -R[0,2]))
		psi = np.rad2deg(np.arctan2(-R[2,1], R[2,0]))
		angles.append([float(phi), float(theta), float(psi)])
	
	return angles
## roll-pitch-yaw
def roll_pitch_yaw(T):
	## angles
	angles = []
	## extracting rotation matrix from T
	if T.shape[0] == 4:
		R = T[:-1,:-1]
	else:
		R = T
	if R[2,0] == 1:
		theta = np.rad2deg(-np.pi/2)
		phi = np.rad2deg(0) #let phi = 0
		psi = np.rad2deg(np.arctan2(R[0,1], R[1,1]))
		angles.append([float(phi), float(theta), float(psi)])
	elif R[2,0] == -1:
		theta = np.rad2deg(np.pi/2)
		phi = np.rad2deg(0) #let phi = 0
		psi = np.rad2deg(np.arctan2(-R[0,1], R[1,1]))
		angles.append([float(phi), float(theta), float(psi)])
	else:
		## first possible solution
		theta = np.rad2deg(np.arctan2(-R[2,0], np.sqrt(R[0,0]**2+R[1,0]**2)))
		phi = np.rad2deg(np.arctan2(R[1,0]/np.cos(np.deg2rad(theta)), R[0,0]/np.cos(np.deg2rad(theta))))
		psi = np.rad2deg(np.arctan2(R[2,1]/np.cos(np.deg2rad(theta)), R[2,2]/np.cos(np.deg2rad(theta))))
		angles.append([float(phi), float(theta), float(psi)])
		## second possible solution
		theta = np.rad2deg(np.arctan2(-R[2,0], -np.sqrt(1-R[2,0]**2)))
		phi = np.rad2deg(np.arctan2(R[1,0]/np.cos(np.deg2rad(theta)), R[0,0]/np.cos(np.deg2rad(theta))))
		psi = np.rad2deg(np.arctan2(R[2,1]/np.cos(np.deg2rad(theta)), R[2,2]/np.cos(np.deg2rad(theta))))
		angles.append([float(phi), float(theta), float(psi)])

	return angles

## end-effector position vector
def ef_position(T):
	return T[:-1,-1]

## end-effector rotational matrix
def ef_rotation(T):
	return T[:-1,:-1]

## forward-kinematics calculator
def forward_kinematics(dh_joints):
	fk = np.eye(4)
	for joint in dh_joints:
		fk = np.dot(fk, dh_to_htm(joint))
	return fk.round(3)

## using geometric approach
def inverse_kinematics_6DOF(tool_pos, tool_orn):
	print(tool_pos,tool_orn)
	## dh parameters for 0,0,0,0,0,0
	dh_para = dh_table([0,0,0,0,0,0])
	## rotational matrix
	R = ef_orientation(tool_orn)
	## wrist center
	p_wrist = (tool_pos - dh_para[-1,-2]*(R@np.array([[0,0,1]]).T).T)[0]
	print(f"Wrist Center: {p_wrist}")
	x_w, y_w, z_w = p_wrist
	## theta one calculation
	q = {'q1': [], 'q2': [], 'q3': [],'q4': [],'q5': [],'q6': []}
	q1 = np.arctan2(y_w,x_w)
	q['q1'].append(q1)
	q['q1'].append(q1+np.pi)
	## calculating theta 3 using cosine law
	## consonants
	r = np.hypot(x_w, y_w)
	s = z_w - dh_para[0,2]
	D = np.hypot(r,s)
	## offsets
	a2 = dh_para[1,0]
	a3 = dh_para[3,2]
	## theta three
	q3 = np.arccos((D**2 - a2**2 - a3**2)/(2*a2*a3))

	## theta two calculation
	beta = 	np.arctan2(s,r)
	gamma = np.arctan2(a3*np.sin(q3), a2 + a3*np.cos(q3))
	## for elbow up: where q3>0
	# q2 = beta - gamma
	## for elbow down: where q3<0
	q2 = beta + gamma

	## calculating the last three joint angels: q4,q5,q6
	## converting rad to degree
	q1 = float(np.rad2deg(q1))
	q2 = float((90-np.rad2deg(q2)).round(3))
	q3 = float((np.rad2deg(q3)-90).round(3))
	print(f"First 3 Joint Angles: {q1,q2,q3}")
	## dh-parameters for the first three joints
	dh_para = dh_table([q1,q2,q3,0,0,0])[:3,:]
	t_03 = forward_kinematics(dh_para)
	## extracting the rotational matrix
	R_03 = ef_rotation(t_03)
	## calculating the last there rotational matrix
	R_36 = np.dot(R_03.T, R)
	zyz = euler_angle_zyz(R_36)
	rpy = roll_pitch_yaw(R_36)
	print(f"Last Three Joint Angles: {zyz}")
	print(f"Last Three Joint Angles: {rpy}")

### Forward Kinematics
joint_angles = 0,0,0,0,0,0
## DH-parameters of 6DOF robot
dh_para = dh_table(joint_angles)
## t06
t_06 = forward_kinematics(dh_para)
## end-effector position
ef_pos = ef_position(t_06)
## end-effector orientation matrix
ef_orn = ef_rotation(t_06)
## euler angles ZYZ
euler = euler_angle_zyz(t_06)
rpy = roll_pitch_yaw(t_06)

print(f"Euler: {euler}")
print(f"RPY: {rpy}")

tool_pos = ef_pos
orn = euler
for i in range(len(orn)):
	print(f"\nUsing: {orn[i]} this angles")
	tool_pos = ef_pos
	tool_orn = orn[i]
	inverse_kinematics_6DOF(tool_pos, tool_orn)