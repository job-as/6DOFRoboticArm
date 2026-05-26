import numpy as np
import sympy as sp

### DH parameters of robot
a2,d1,d4,d6 = sp.symbols('a2 d1 d4 d6')
q1,q2,q3,q4,q5,q6 = sp.symbols('q1 q2 q3 q4 q5 q6')

def transMatrix(a,alpha,d,theta):
	r_z = sp.Matrix(np.array([
			[sp.cos(theta), -sp.sin(theta), 0, 0],
			[sp.sin(theta), sp.cos(theta), 0, 0],
			[0, 0, 1, 0],
			[0, 0, 0, 1]
		]))
	t_z = sp.Matrix([
			[1,0,0,0],
			[0,1,0,0],
			[0,0,1,d],
			[0,0,0,1]
		])
	t_x = sp.Matrix([
			[1,0,0,a],
			[0,1,0,0],
			[0,0,1,0],
			[0,0,0,1]
		])
	r_x = sp.Matrix([
			[1,0,0,0],
			[0,sp.cos(alpha), -sp.sin(alpha),0],
			[0,sp.sin(alpha), sp.cos(alpha),0],
			[0,0,0,1]
		])

	print(sp.simplify(r_z*t_z*t_x*r_x))
	print(sp.simplify(r_x*t_x*t_z*r_z))

# a_1 = transMatrix(0,-sp.pi/2,d1,q1)
# a_2 = transMatrix(a2,0,0,q2-sp.pi/2)
# a_3 = transMatrix(0,-sp.pi/2,0,q3)
# a_4 = transMatrix(0,sp.pi/2,d4,q4)
# a_5 = transMatrix(0,-sp.pi/2,0,q5)
# a_6 = transMatrix(0,0,d6,q6+sp.pi/2)

# a_06 = sp.simplify(a_1*a_2*a_3*a_4*a_5*a_6)
# print(a_06)
# print(a_06.subs({q1:0, q2:0, q3:0, q4:0, q5:0, q6:0, d1:1, d4:2, d6:1, a2: 1}))

a, al, d, th = sp.symbols('a alpha d theta')

zz = transMatrix(a,al,d,th)
print(zz)