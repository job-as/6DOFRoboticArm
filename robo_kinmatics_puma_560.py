import numpy as np
import pandas as pd
def dh_to_htm(a, alpha, d, theta):
	## converting degree to radian
	theta = (np.pi/180)*theta
	alpha = (np.pi/180)*alpha
	## homogeneous transformation matrix for a joint
	T = np.array([
			[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
			[np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
			[0, np.sin(alpha), np.cos(alpha), d],
			[0, 0, 0, 1]
		])
	return T.round(3)

## forward kinematics calculation
def forward_kinamatics_dh(dh_table):
	fk = np.eye(4)
	for joint in dh_table:
		fk=np.dot(fk,dh_to_htm(*joint))
	return fk.round(3)

## formatting the dh parameters
def dh_parameters(*q):
	q1,q2,q3,q4,q5,q6 = q
	dh_table = np.array([
				[0,-90,1,q1],
				[1,0,0,-q2+90],
				[1,90,0,q3-90],
				[1,-90,0,q4],
				[0,-90,0,-q5+90],
				[0,0,1,q6+90],
			])
	print(dh_table)
	return dh_table

dh_table = dh_parameters(0,10,0,0,0,0)
# t03 = forward_kinamatics_dh(dh_table[:3,:])
t04 = forward_kinamatics_dh(dh_table[:4,:])
t36 = forward_kinamatics_dh(dh_table[3:,:])
t06 = forward_kinamatics_dh(dh_table)

R = t06[:-1,:-1]
end_pos = t06[:-1,-1]
OC = t04[:-1,-1]
# print(f"\nwist position: {t03[:-1,-1]}")
# print(f"\nwist position: {t04[:-1,-1]}")
# print(f"\nend-effector position from Oc: {ef_pos}")
print(f"\nend-effector position: {end_pos}")
print(f"\nRotational Matrix\n {R}")
# print(f"\nRotational Matrix\n {t06[:-1,:-1]}")

# ## extracting the angle from r
# if (R[2,0].round(1)==0) and (R[2,1].round(1)==0):
# 	theta = 0 if R[2,2].round(1)==1 else np.pi
# 	phi = 0
# 	psi = np.arctan2(R[0,1], R[0,0])
# else:
# 	if end_pos[-1] < 4:
# 		theta = np.arctan2(R[2,2],-np.sqrt(1-R[2,2]**2))
# 		phi = np.arctan2(-t06[1,2], -t06[0,2]).round(3)
# 		psi = np.arctan2(-t06[2,1], t06[2,0]).round(3)
# 	else:
# 		theta = np.arctan2(R[2,2],np.sqrt(1-R[2,2]**2))
# 		phi = np.arctan2(t06[1,2], t06[0,2]).round(3)
# 		psi = np.arctan2(t06[2,1], -t06[2,0]).round(3)

# print(np.round(phi*180/np.pi,1), np.round(theta*180/np.pi,1), np.round(psi*180/np.pi,1))

# def inverse_kinamatics(x,y,z,q4,q5,q6):
# 	a2, a3 = 1,1
# 	wrist_center = np.array([[x,y,z-2]]) - 2*np.dot(R, np.array([[0,0,1]]).T).T
# 	print(wrist_center)
# 	## theta one calculation
# 	if x >= 0 or y >= 0:
# 		q1 = np.arctan2(y,x)
# 	else:
# 		q1 = np.pi + np.arctan2(y,x)
# 	## theta three calculation
# 	D = (x**2 + y**2 + z**2 - a2**2 - a3**2)/2*a2*a3
# 	print(f"D:{D}")
# 	q3 = np.arctan2(np.sqrt(1-D**2), D)
# 	## theta two calculation
# 	q2 = np.arctan2(z,np.sqrt(x**2+y**2))- np.arctan2(a3*np.sin(q3),a2+a3*np.cos(q3))

# 	print(q1*180/np.pi,q2*180/np.pi,q3*180/np.pi)

# x,y,z = end_pos
# inverse_kinamatics(x,y,z,0,0,0)
# print(OC)