%% arc path generator
function arc_points = arc_path_generator(t_points, arc_type, N)
	%% de-structuring
    p_1 = t_points(1,:);
    p_2 = t_points(2,:);
    p_3 = t_points(3,:);
	%% finding the plane of the circle
	%% line vectors
	v_12 = p_2 - p_1;
	v_23 = p_3 - p_2;
	%% normal vector
	v_normal = cross(v_12, v_23);
	%% magnitude of the vector
	m_normal = norm(v_normal);
	%% checking for collinearity of the points
	% if m_normal > 1e-8
	% 	return
	%% normalizing the normal vector
	vn_normal = v_normal/m_normal;
	%% creating the new plane axis
	%% making v_12 as the new x-axis
	x_new = v_12/norm(v_12);
	y_new = cross(vn_normal, x_new);
	y_new = y_new/norm(y_new);
	%% projecting the new points to the new plane with origin at p_1
	%% projecting from 3D points to 2D points
	%% the new 2D points
	p_1n = [dot((p_1 - p_1),x_new), dot((p_1 - p_1), y_new)];
	p_2n = [dot((p_2 - p_1),x_new), dot((p_2 - p_1), y_new)];
	p_3n = [dot((p_3 - p_1),x_new), dot((p_3 - p_1), y_new)];
	%% find 2D circle center using perpendicular bisector intersection
	%% lines midpoints
	mid_12 = (p_1n+p_2n)/2;
	mid_23 = (p_2n+p_3n)/2;
	%% print(mid_12, mid_23)
	%% slope of perpendicular bisectors
	d_12 = p_2n - p_1n;
	d_23 = p_3n - p_2n;
	%% print(d_12, d_23)
	%% matrix inversion method Ax = b
	A = [d_12; d_23];
	b = [d_12*mid_12', d_23*mid_23'];
	%% circle center point
	center = A\b';
    %% raduis of the circle
    radius = norm((p_1n-center'));
	%% solving for angles of the three points around center
	angles = [
        atan2(p_1n(2)-center(2), p_1n(1)-center(1)),
        atan2(p_2n(2)-center(2), p_2n(1)-center(1)),
        atan2(p_3n(2)-center(2), p_3n(1)-center(1))
    ];
    th1 = angles(1); th2 = angles(2); th3 = angles(3);
	%% making the angle from 0 to 360
    angles = mod(angles,2*pi);
    th1n = angles(1); th2n = angles(2); th3n = angles(3);
	%% print(th1n,th2n,th3n)
	%% decide sweep direction so arc passes through P2
	dir_13 = mod((th3n - th1n),2*pi);
	dir_12 = mod((th2n - th1n),2*pi);
	%% select types of arc
	if arc_type == 'a'
		if dir_12 <= dir_13
			%% short CCW arc from th1 to th3 contains th2
		    thetas = th1 + linspace(0, dir_13, N);
        else
			%% long way (clockwise) from a1 to a3: represent as negative sweep
		    thetas = th1 + linspace(0, dir_13 - 2*pi, N);
        end
	else
		if dir_12 <= dir_13
			thetas = th1 - linspace(0,2*np.pi, N);
        else
			thetas = th1 + linspace(0,2*pi, N);
        end
    end
	%% compute 2D arc points
	pts2 = [center(1) + radius*cos(thetas); center(2) + radius*sin(thetas)]';
	%%  the arc points in 3D
    arc_points = zeros(N,3);
    for i=1:size(pts2,1)
        arc_points(i,:) = p_1 + pts2(i,1) * x_new +pts2(i,2) * y_new;
    end
	%% center in 3D
	center3d = p_1 + center(1) * x_new + center(2) * y_new;
    %% returning the result
end
close all; clear all; clc;
t_points = [-1,0,0;0,1,0;1,0,0];
arc_points = arc_path_generator(t_points, 'a', 20);
plot(arc_points(:,1),arc_points(:,2))