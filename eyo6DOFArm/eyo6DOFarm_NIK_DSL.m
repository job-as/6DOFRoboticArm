close all; clear all; clc;
%% DH - parameters
function dh_paras = dh_parameters(joint_ang)
	%% number of joints
    no_joints = size(joint_ang,2);
    %% dh parameters size
    dh_paras = zeros(no_joints,4); j_angles = zeros(1,6);
    if (no_joints < 6)
        j_angles = [joint_ang, zeros(1,6-no_joints)];
    else
        j_angles = joint_ang;
    end
	%% DH - parameters matrix
	dh = [
            [0.045,  -pi/2,  0.221,   j_angles(1)];
            [0.320,   0,     0.000,   j_angles(2)-pi/2];
            [0.000,  -pi/2,  0.000,   j_angles(3)];
            [0.000,   pi/2,  0.218,   j_angles(4)];
            [0.000,  -pi/2,  0.000,   j_angles(5)];
            [0.000,   0,     0.081,   j_angles(6)-pi/2]
       ];
    dh_paras = dh(1:no_joints,:);
end

%% DH to Homogeneous transformation matrix
function t = dh_to_htm(dh)
	%% abbreviations
	c_theta = cos(dh(4)); s_theta = sin(dh(4));
	c_alpha = cos(dh(2)); s_alpha = sin(dh(2));
	%% Homogeneous transformation matrix
	t = [
			[c_theta, -s_theta*c_alpha, s_theta*s_alpha, dh(1)*c_theta],
			[s_theta, c_theta*c_alpha, -c_theta*s_alpha, dh(1)*s_theta],
			[0, s_alpha, c_alpha, dh(3)],
			[0, 0, 0, 1]
		];
end
%% forward_kinematics: Implementation Forward kinematics
function htm = forward_kinematics(j_angles)
	%% generating DH-parameters
	joints = dh_parameters(j_angles);
	%% solution holder
	htm = eye(4);
	for r=1:size(joints,1)
		htm = htm*dh_to_htm(joints(r,:));
    end
end
%% geometric_jacobian: computing Geometric Jacobian matrix
function jac = geometric_jacobian(j_angles)
	%% the first joint axis(z_0) and relative position(p_0) 
	z_0 = [0,0,1]'; p_0 = [0,0,0]';
	%% forward kinematics for given joint angles
	eef_pos = forward_kinematics(j_angles);
    eef_pos = eef_pos(1:3,4);
	%% computing Jacobian matrix for given joint angles 
	%% linear(j_v) and angular (j_w) velocity
	jv = zeros(3,6); jw = zeros(3,6);
	for j=1:size(j_angles,2)
		%% homogeneous transformation matrix
		htm = forward_kinematics(j_angles(1:j));
		%% linear velocity component
		jv(:,j) = (cross(z_0, (eef_pos-p_0)));
		%% angular velocity component
		jw(:,j) = (z_0);
		%% axis of rotational matrix
		z_0 = htm(1:3,3);
		%% joint j origin or relative position
		p_0 = htm(1:3,4);
    end
	%% merging the two components
	jac = [jv;jw];
end

%% pos_error: computing position error
function err = pos_error(p_d, p_e)
	%% position error
	err = (p_d - p_e);
end
%% orn_error: computing orientation error
function err = orn_error(R_d, R_e)
	%% de-structuring n, s, a
	n_d = R_d(:,1); s_d = R_d(:,2); a_d = R_d(:,3);
	n_e = R_e(:,1); s_e = R_e(:,2); a_e = R_e(:,3);
	%% orientation error
	err = 0.5*(cross(n_e,n_d) + cross(s_e,s_d) + cross(a_e,a_d));
end
%% error_norm: computing error and norm
function [err, e_norm] = error_norm(t_pose, j_angles)
	%% current pose based on current j-angles
	c_pose = forward_kinematics(j_angles);
	%% error in position
	e_pos = pos_error(t_pose(1:3,4), c_pose(1:3,4));
	%% error in orientation
	e_orn = orn_error(t_pose(1:3,1:3), c_pose(1:3,1:3));
	%% merging the errors
	err = [e_pos; e_orn];
	%% error norm(magnitude)
	e_norm = norm(err);
end
%% dls_inverse_kinematics: DLS algorithm for IK
function j_angles = dls_inverse_kinematics(t_pose, q0)
    lamda=0.1; tol=1e-15; max_ite=1000;
	%% setting initial joint angles
	j_angles = q0;
	%% error and norm
	[err, e_norm] = error_norm(t_pose, j_angles);
    %% making iteration
    for i=1:max_ite
	    %% computing the geometric Jacobian matrix 
	    %% for the current joint angles
	    J = geometric_jacobian(j_angles);
	    %% computing change in delta using DLS
	    delta = (J'*pinv(J*J' + (lamda^2).*eye(size(J,1)))*err)';
	    %% updating joint angles
	    j_angles = j_angles + clip(delta, -0.5,0.5);
	    %% recomputing error
	    p_e_norm = e_norm;
	    %% updating error
	    [err, e_norm] = error_norm(t_pose, j_angles);
	    %% breaking conditon
	    if (e_norm < tol)
		    break
        end
	    %% updating lamda
	    lamda = lamda*(e_norm/p_e_norm);
    end
end
%% orn_generator: tool orientation generator
function orn = orn_generator(j_angles)
    %% adding the three joint angles
    j_angles = [0,0,0,j_angles];
    %% computing t_06
    t_06 = forward_kinematics(j_angles);
    %% extracting orinetation
    orn = t_06(1:3,1:3);
end
%% inputGenerator: function description
function output = inputGenerator(t_points, T, N)
    %% adding the first point on last to make loop
	%t_points = [[0.322,0,0.541]; t_points]
    %% time vector
	t = linspace(0, T, N)';  
	% Time per segment
	segments = size(t_points,1) - 1;   % 4 sides
	Tseg = T / segments;
	% Allocate path
	x = linspace(t_points(1,1), t_points(2,1), N);
	y = linspace(t_points(1,2), t_points(2,2), N);
	z = linspace(t_points(1,3), t_points(2,3), N);
	%% formatting output
    output = [t, x', y', z'];
end
%% linear_path_generator: linear path generator
function t_pos = linear_path_generator(t_points, type, T, N)
    %% adding the first point on last to make loop
	if type ~= 'o'
		t_points = [t_points;t_points(1,:)];
	end
    %% time vector
	t = linspace(0, T, N)';  
	% Time per segment
	segments = size(t_points,1) - 1;   % n sides
	Tseg = T / segments;
	% Allocate path
	x = zeros(N,1);
	y = zeros(N,1);
	z = zeros(N,1);
	% Generate trajectory
	for k = 1:segments
	    % time indices for this segment
	    idx = t >= (k-1)*Tseg & t <= k*Tseg;
	    % normalize local time
	    tau = (t(idx) - (k-1)*Tseg) / Tseg;
	    % linear interpolation between points
	    x(idx) = (1-tau)*t_points(k,1) + tau*t_points(k+1,1);
	    y(idx) = (1-tau)*t_points(k,2) + tau*t_points(k+1,2);
	    z(idx) = (1-tau)*t_points(k,3) + tau*t_points(k+1,3);
	end
	%% formatting output
    t_pos = [t, x, y, z];
end
%% circle_generator: circle path generator
function [t_pos] = circle_generator(r, h, offset, T, N)
    %% fixing size
    t_pos = zeros(N,4);
    %% time vector
	t = linspace(0, T, N)';
    %% omwega
    omega = 2*pi/T;
    %% points
    x = offset + r*cos(omega*t);
    y = offset + r*sin(omega*t);
    z = h*ones(size(t));
	%% positon matrix
    t_pos = [t, x, y, z];   
end
%% arc_generator: arc path generator
function t_pos = arc_path_generator(t_points, arc_type, N, T)
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
			thetas = th1 - linspace(0,2*pi, N);
        else
			thetas = th1 + linspace(0,2*pi, N);
        end
    end
	%% compute 2D arc points
	pts2 = [center(1) + radius*cos(thetas); center(2) + radius*sin(thetas)]';
	%% center in 3D
	center3d = p_1 + center(1) * x_new + center(2) * y_new;
    %% arc points
    t_pos = zeros(N,4);
    %% setting the first column to time
    t_pos(:,1) = linspace(0,T, N)';
	%%  computing arc points in 3D
    for i=1:size(pts2,1)
        t_pos(i,2:4) = p_1 + pts2(i,1) * x_new +pts2(i,2) * y_new;
    end
end
%% n-side polygon creator
function t_pos = nSidePolygon(n_side, r, j_angles, T, N)
	%% solution holder
	t_pos = zeros(N,4);
	%% polygon vertices
	xyz = zeros(n_side,3);
	%% current position
	xyz(1,:) = posExtractor(j_angles);
	%%  solving for the n vertexes
	for j=1:n_side-1
		%% position
		xyz(j+1,:) = [xyz(j,1) + r*cos((j-1)*2*pi/n_side), xyz(j,2) + r*sin((j-1)*2*pi/n_side), xyz(j,3)];
    end
	%% generating linear path between vertexes
	t_pos = linear_path_generator(xyz, 'c', T, N);
end
%% joint_angle_generator: joint angle generator
function j_angles = joint_angle_generator(t_pos, t_orn, T, N)
    %% time
    t = linspace(0,T, N);
    %% joint angle generation
    j_angles = zeros(N, 7);
    j_angles(:,1) = t;
    ang = [0,0,0,0,0,0];
    for i=1:N
        t_pose = [t_orn,t_pos(i,2:4)';0,0,0,1];
        ang = dls_inverse_kinematics(t_pose, ang);
        j_angles(i,2:7) = ang;
    end
end
%% posExtractor: position extractor
function [t_pos] = posExtractor(j_angles)
	t_pos = zeros(size(j_angles,1),3);
	%% computing forward kinematics
	for i=1:size(j_angles,1)
	   htm = forward_kinematics(j_angles(i,2:7));
	   t_pos(i,:) = round(htm(1:3,4),4);
	end
end
%% plotJointAngles: function description
function plotJointAngles(j_angles)

	figure
	plot(j_angles(:,1), j_angles(:,2))
	hold on
	plot(j_angles(:,1), j_angles(:,2))
	plot(j_angles(:,1), j_angles(:,3))
	plot(j_angles(:,1), j_angles(:,4))
	plot(j_angles(:,1), j_angles(:,5))
	plot(j_angles(:,1), j_angles(:,6))
	plot(j_angles(:,1), j_angles(:,7))
	grid on
	legend('q1','q2','q3','q4','q5','q6')
end
%% plotAllInOne: plotting input and output paths, and joint angles
function plotAllInOne(in_pos, out_pos, j_angles)
    %% data formating
    in_pos = round(in_pos,4);
	%% Create a 3×3 grid layout
	t = tiledlayout(3,3, 'Padding','compact', 'TileSpacing','compact');
	%% Input path plots
	%% top view
	nexttile;
	plot(in_pos(:,2), in_pos(:,3)); title('Input Top-View');     xlabel('X(m)'); ylabel('Y(m)');
	grid on;
	%% front view
	nexttile;
	plot(in_pos(:,2), in_pos(:,4)); title('Input Front View');   xlabel('X(m)'); ylabel('Z(m)');
	grid on;
	%% side view
	nexttile;
	plot(in_pos(:,3), in_pos(:,4)); title('Input Side View');    xlabel('Y(m)'); ylabel('Z(m)');
	grid on;
	%% Output path plots
	%% top view
	nexttile;
	plot(out_pos(:,1), out_pos(:,2)); title('Output Top-View');   xlabel('X(m)'); ylabel('Y(m)');
	grid on;
	%% front view
	nexttile;
	plot(out_pos(:,1), out_pos(:,3)); title('Output Front View'); xlabel('X(m)'); ylabel('Z(m)');
	grid on;
	%% side view
	nexttile;
	plot(out_pos(:,2), out_pos(:,3)); title('Output Side View');  xlabel('Y(m)'); ylabel('Z(m)');
	grid on;
	%% Joint angles Vs time
	nexttile(7, [1 3]);   % [rows columns]
	plot(j_angles(:,1), j_angles(:,2))
	hold on
	for j=2:7
		plot(j_angles(:,1), j_angles(:,j))
	end
	grid on
	legend('q1','q2','q3','q4','q5','q6')
	title('Joint Angles vs Time'); xlabel('Time (s)'); ylabel('Angle (rad)');
end
%% path_generator: input path generator
function  j_angles = path_generator(type)
	%% Number of points
    N = 100;
    %% simulation time
    T = str2double(get_param(eyo6DOFarmModel, "StopTime"));

    if type == 'lino3'
	    %% linear open path: three points input
	    %% target positions
	    t_points = [0.3,0,0.2; 0.3,0, 0.6;0.5,0, 0.6;];
	    %% target orientation
	    t_orn = orn_generator([0,0,0]);
	    %% linear path
	    in_pos = linear_path_generator(t_points, 'o', T, N);
	    %% joint angle generation
	    j_angles = joint_angle_generator(in_pos, t_orn, T, N);
	    %% output position
	    out_pos = posExtractor(j_angles);
        %% plotting the results
        plotAllInOne(in_pos, out_pos, j_angles);

	elseif type == 'linc3'
	    %% linear closed path: three points input: triangle
	    %% target positions
	    t_points = [0.3,0.1,0.2; 0.3,0.3, 0.4;0.5,0.1, 0.4;];
	    %% target orientation
	    t_orn = orn_generator([0,0,0]);
	    %% linear path
	    in_pos = linear_path_generator(t_points, 'c', T, N);
	    %% joint angle generation
	    j_angles = joint_angle_generator(in_pos, t_orn, T, N);
	    %% output position
	    out_pos = posExtractor(j_angles);
        %% plotting the results
        plotAllInOne(in_pos, out_pos, j_angles);

	elseif type == 'linc4'
	    %% linear closed path: four points input: rectangle
	    %% target positions
	    t_points = [0.3,0,0.2; 0.3,0, 0.6;0.5,0, 0.6;0.5,0, 0.2];
	    %% target orientation
	    t_orn = orn_generator([0,0,0]);
	    %% linear path
	    in_pos = linear_path_generator(t_points, 'c', T, N);
	    %% joint angle generation
	    j_angles = joint_angle_generator(in_pos, t_orn, T, N);
	    %% output position
	    out_pos = posExtractor(j_angles);
        %% plotting the results
        plotAllInOne(in_pos, out_pos, j_angles);

	elseif type == 'circr'
	    %% circular path: from center point and radius
	    in_pos = circle_generator(0.1, 0.2, 0.3, T, N);
        %% target orientation
	    t_orn = orn_generator([0,0,0]);
	    %% joint angle generation
	    j_angles = joint_angle_generator(in_pos, t_orn, T, N);
	    %% output position
	    out_pos = posExtractor(j_angles);
        %% plotting the results
        plotAllInOne(in_pos, out_pos, j_angles);
	elseif type == 'arc3p'
        %% arc path: from three points
		%% target positions
	    t_points = [0.3,0,0.2; 0.3,0, 0.4;0.5,0, 0.4];
        %% target orientation
	    t_orn = orn_generator([0,0,0]);
		%% arc path: from three points
	    in_pos = arc_path_generator(t_points, 'a', N, T);
	    %% joint angle generation
	    j_angles = joint_angle_generator(in_pos, t_orn, T, N);
	    %% output position
	    out_pos = posExtractor(j_angles);
        %% plotting the results
        plotAllInOne(in_pos, out_pos, j_angles);

    elseif type == 'cir3p'
        %% cicular path: from three points
		%% target positions
	    t_points = [0.3,0,0.2; 0.3,0, 0.4;0.5,0, 0.4];
        %% target orientation
	    t_orn = orn_generator([0,0,0]);
		%% arc path: from three points
	    in_pos = arc_path_generator(t_points, 'c', N, T);
	    %% joint angle generation
	    j_angles = joint_angle_generator(in_pos, t_orn, T, N);
	    %% output position
	    out_pos = posExtractor(j_angles);
        %% plotting the results
        plotAllInOne(in_pos, out_pos, j_angles);

    elseif type == 'nside'
        %% n-side polygon: from starting point and raduies
		%% target positions
	    t_points = [0.3,0,0.2; 0.3,0, 0.4;0.5,0, 0.4];
        %% target orientation
	    t_orn = orn_generator([0,0,0]);
		%% arc path: from three points
	    in_pos = nSidePolygon(6, 0.1, [0,0,pi/2,0,0,0,0], T, N);
        %% joint angle generation
	    j_angles = joint_angle_generator(in_pos, t_orn, T, N);
	    %% output position
	    out_pos = posExtractor(j_angles);
        %% plotting the results
        plotAllInOne(in_pos, out_pos, j_angles);
	end
end

%% linear open path: three points input
% j_angles = path_generator('lino3');

%% linear closed path: three points input: triangle
% j_angles = path_generator('linc3');

%% linear closed path: four points input: rectangle
% j_angles = path_generator('linc4');

%% circular path: from center point and radius
% j_angles = path_generator('circr');

%% arc path: from three points
% j_angles = path_generator('arc3p');

%% cicular path: from three points
j_angles = path_generator('cir3p');

%% n-side polygon: from starting point and raduies
% j_angles = path_generator('nside');