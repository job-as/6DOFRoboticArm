import numpy as np

def homo_matrix(theta, p_borgi):
	theta = (np.pi/180)*theta
	## rotational matrix
	r_z = np.array([
		[np.cos(theta), -np.sin(theta), 0],
		[np.sin(theta), np.cos(theta), 0],
		[0,0,1]])

	r_x = np.array([
		[1,0,0],
		[0, np.cos(theta), -np.sin(theta)],
		[0, np.sin(theta), np.cos(theta)]])

	r_y = np.array([
		[np.cos(theta),0, np.sin(theta)],
		[0,1,0],
		[-np.sin(theta),0, np.cos(theta)]])

	## homogeneous transformation matrix
	t_z = np.concat((r_z,p_borgi.T), axis=1)
	t_z = np.concat((t_z, np.array([[0,0,0,1]])), axis=0)
	t_y = np.concat((r_y,p_borgi.T), axis=1)
	t_y = np.concat((t_y, np.array([[0,0,0,1]])), axis=0)
	t_x = np.concat((r_x,p_borgi.T), axis=1)
	t_x = np.concat((t_x, np.array([[0,0,0,1]])), axis=0)

	return t_x, t_y, t_z

## examples
p_borgi = np.array([[10,5,0]])
theta = 30
p_b = np.array([[3.0,7.0,0,1]])

_,_, t = homo_matrix(theta, p_borgi)
p_a = np.round(np.dot(t, p_b.T)[:-1,:],3)

## example
p_borgi = np.array([[4,3,0]])
theta = 30
_,_, t = homo_matrix(theta, p_borgi)
t_inv = np.linalg.inv(t)
tx,_,_ = homo_matrix(theta, p_borgi)
# print(tx)

a = np.array([[1,2,3]])
b = np.array([[2,3,4]])

print(np.dot(a.T,b))