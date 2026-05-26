import numpy as np
import sympy as sp

def degree_2_rad(theta):
	return (np.pi/180)*theta

def dh_2_matrix(a,alpha,d,theta):

	rot_z = sp.Matrix([
			[sp.cos(theta), -sp.sin(theta), 0, 0],
			[sp.sin(theta), sp.cos(theta), 0, 0],
			[0, 0, 1, 0],
			[0, 0, 0, 1]
		])
	tra_z = sp.Matrix([
			[1,0,0,0],
			[0,1,0,0],
			[0,0,1,d],
			[0,0,0,1]
		])
	tra_x = sp.Matrix([
			[1,0,0,a],
			[0,1,0,0],
			[0,0,1,0],
			[0,0,0,1]
		])
	rot_x = sp.Matrix([
			[1, 0, 0, 0],
			[0, (sp.cos(alpha)), -(sp.sin(alpha)), 0],
			[0, (sp.sin(alpha)), (sp.cos(alpha)), 0],
			[0, 0, 0, 1]
		])
	return sp.simplify(rot_z*tra_z*tra_x*rot_x)
# a2, d1,d4, d6 = sp.symbols('a2, d1 d4 d6')
# q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')
a,alpha,d, theta = sp.symbols('a alpha d theta')
print(dh_2_matrix(a, alpha, d, theta))
# a_1 = dh_2_matrix(0,np.pi/2, d1, q1)
# a_2 = dh_2_matrix(a2,0, 0, q2+np.pi/2)
# a_3 = dh_2_matrix(0,-np.pi/2, 0, q3-np.pi/2)
# a_4 = dh_2_matrix(0,np.pi/2, d4, q4)
# a_5 = dh_2_matrix(0,-np.pi/2, 0, q5)
# a_6 = dh_2_matrix(0,0, d6, q6)

# def hom_tarns_matrix(*args):
# 	res = sp.Matrix([
# 			[1,0,0,0],
# 			[0,1,0,0],
# 			[0,0,1,0],
# 			[0,0,0,1]
# 		])
# 	for x in args:
# 		res=sp.simplify(res*x)
# 	return res


# t_03 = hom_tarns_matrix(a_1, a_2, a_3)
# t_36 = hom_tarns_matrix(a_4, a_4, a_6)
# t_06 = hom_tarns_matrix(t_03, a_4, a_5, a_6)

# final_pos = t_06.subs({q1:0,q2:0,q2:0,q3:0,q4:0,q5:0,q6:0,d1:1,d4:2,d6:2, a2:1})
# final_pos = t_06.subs({q1:0,q2:0,q2:0,q3:0,q4:0,q5:0,q6:0,d1:1,d4:2,d6:2, a2:1})
# orn = np.array(final_pos_orn[:-1,:-1])
# pos = np.array(final_pos_orn[:-1,-1])
# print(orn.shape)
# print(pos.shape)
