import numpy as np

def degree_to_rad(angle):
	return (np.pi*angle/180).round(3)
def rad_to_degree(angle):
	return round((180*angle/np.pi))

def dh_to_htm(parameters):
	a, alpha, d, theta = parameters
	alpha = degree_to_rad(alpha)
	theta = degree_to_rad(theta)
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

def forward_kinematics(joints):
	fk = np.eye(4)
	for joint in joints:
		fk = np.dot(fk, dh_to_htm(joint))
	return fk.round(3)

def ef_orientation(orn):
	phi, theta, psi = orn
	## degree to rad conversion
	phi = degree_to_rad(phi)
	theta = degree_to_rad(theta)
	psi = degree_to_rad(psi)
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

## end-effector position
def tool_position(T):
	return T[:-1,-1]

## end-effector rotational matrix
def tool_rotation(T):
	return T[:-1,:-1]

### Euler angles calculation for ZYZ configuration
def euler_angle_zyz(T):
	## extracting rotation matrix from T
	if T.shape[0] == 4:
		R = T[:-1,:-1]
	else:
		R = T
	## if r_33 != +-1
	if R[2,2] != 1 and R[2,2] != -1:
		## first possible solution
		theta = np.arctan2(np.sqrt(1-R[2,2]**2), R[2,2])
		phi = np.arctan2(R[1,2], R[0,2])
		psi = np.arctan2(R[2,1], -R[2,0])

		## second possible solution
		theta = np.arctan2(-np.sqrt(1-R[2,2]**2), R[2,2])
		phi = np.arctan2(-R[1,2], -R[0,2])
		psi = np.arctan2(-R[2,1], R[2,0])
	elif R[2,2] == 1:
		theta = 0
		## sum of phi and psi can be calculated
		phi = 0  #let phi = 0
		psi = np.arctan2(R[1,0],R[0,0])
	elif R[2,2] == -1:
		theta = 0
		## sum of phi and psi can be calculated
		phi = 0  #let phi = 0
		psi = np.arctan2(-R[0,1],-R[0,0])

	phi = rad_to_degree(phi)
	theta = rad_to_degree(theta)
	psi = rad_to_degree(psi)
	## end-effector orientation in degrees
	r = np.array([phi, theta, psi], dtype= 'int64')
	
	return r.round(3)

## using geometric approach
def inverse_kinematics_6DOF(tool_pos, tool_orn):
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
	q1 = rad_to_degree(q1)
	q2 = 90-rad_to_degree(q2)
	q3 = rad_to_degree(q3)-90
	print(f"First 3 Joint Angles: {q1,q2,q3}")
	## dh-parameters for the first three joints
	dh_para = dh_table([q1,q2,q3,0,0,0])[:3,:]
	t_03 = forward_kinematics(dh_para)
	## extracting the rotational matrix
	R_03 = tool_rotation(t_03)
	## calculating the last there rotational matrix
	R_36 = np.dot(R_03.T, R)
	zyz = euler_angle_zyz(R_36)
	q4,q5,q6 = zyz[0],zyz[1],zyz[2]-90
	print(f"Last Three Joint Angles: {q4,q5,q6}")
########################################################################
########################################################################
########################################################################
## Forward kinematics calculation
print("\nForward Kinematics")
## joint angles
thetas = 30,45,0,0,0,0
## dh parameters
dh_para = dh_table(thetas)
# print(dh_para)
## transformation matrix
t_06 = forward_kinematics(dh_para)
t_03 = forward_kinematics(dh_para[:3,:])
t_04 = forward_kinematics(dh_para[:4,:])
# print(t_03)
# print(t_04[:-1,-1])
# print(t_06)
### end-effector position
p_end = tool_position(t_06)
### end-effector orientation
r_end = euler_angle_zyz(t_06)

print(f"end-effector position: {p_end}")
print(f"end-effector orientation: {r_end}")
print(f"Wrist Center: {t_04[:-1,-1]}")

print("\nInverse Kinematics")
inverse_kinematics_6DOF(p_end, r_end)