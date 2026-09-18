
%% inverse kinematics
function q_one = inverse_kinematics(traget_orn, target_pos, qi)
    %%% q_one
    q_one = zeros(1,6);
	%% DH-parameters constants
	dh_const = dh_parameters([0,0,0,0,0,0]);
	a2 = dh_const(2,1); d4 = dh_const(4,3);
	d1 = dh_const(1,3); d6 = dh_const(6,3);
	%% wrist center calculation backwards from end-effector
	w_center = target_pos' - d6*traget_orn(:,3)';
	%% de-structuring x, y, z points
	wc_x = w_center(1); wc_y = w_center(2); wc_z = w_center(3);
	%% calculating joint angle using decoupling method
	%% calculating q_1,q_2 and q_3 first
	%% theta one (q_1): right and left shoulder
	q_1 = [-atan2(wc_y, wc_x), pi - atan2(wc_y, wc_x)];
	%% theta three elbow up and down
	%% the hypotenuses of x and y
	r = hypot(wc_x, wc_y);
	%% height above d_1
	s = wc_z - d1;
	%% using cosine law
	D = (r^2+s^2-a2^2-d4^2)/(2*a2*d4);
    if abs(D) <= 1
		%% q_3 for elbow up and down
		q_3 = [atan2(sqrt(1-D^2), D), atan2(-sqrt(1-D^2), D)];
		%% theta two for each theta one and three
		%% for +ve s_3 and -ve s_3
		m1 = [a2 + d4*cos(q_3(1)), a2 + d4*cos(q_3(2))];
		m2 = [d4*sin(q_3(1)), d4*sin(q_3(2))];
		%% theta two
        q_2 = [
				atan2((m1(1)*s - m2(1)*r), (m1(1)*r + m2(1)*s)); %% s3 +ve
				atan2((m1(1)*s + m2(1)*r), (-m1(1)*r + m2(1)*s));%% s3 +ve
				atan2((m1(2)*s - m2(2)*r), (m1(2)*r + m2(2)*s)); %% s3 -ve
				atan2((m1(2)*s + m2(2)*r), (-m1(2)*r + m2(2)*s));%% s3 -ve
			] - pi/2;
        %% adjusting
        q_3 = q_3 -[pi/2, -pi/2];
        %% all posible solutions
        qs = [
			%% right shoulder
			%% elbow up
			q_1(1), q_2(1), q_3(1);
			%% elbow down
			q_1(1), q_2(3), q_3(2);
			%% left shoulder
			%% elbow up
			q_1(2), q_2(2), q_3(1);
			%% elbow down
			q_1(2), q_2(4), q_3(2);
		];
        %% position checking
        qs = position_checking(qs,w_center);
        %% all possible solution
        q_sol = zeros(4,6); r_idx = 1;
        %% solving for the rest joint angles
        for j=1:size(qs,1)
            %% DH parameter for the 3 joints
			dh = dh_parameters([qs(j,1), qs(j,2), qs(j,3), 0, 0, 0]);
            %% rotational matrix for the first 3 joints
			R_03 = orn_extarction(forward_kinematics(dh(1:3,:)));
            %% rotational matrix for the last 3 joints
			R_36 = (R_03')*(traget_orn);
			%% using the ZYZ configuration
			[q_456_1, q_456_2] = zyz_method(R_36);
			%% adding all possible solutions
			if ~isempty(q_456_1) 
                q_sol(r_idx,:)= [qs(j,1), qs(j,2), qs(j,3), -q_456_1(1), q_456_1(2),-q_456_1(3)];
                r_idx = r_idx+1;
            end
			if ~isempty(q_456_2) 
                q_sol(r_idx,:)= [qs(j,1), qs(j,2), qs(j,3), -q_456_2(1), q_456_2(2),-q_456_2(3)];
                r_idx = r_idx+1;
            end
        end
        %% selecting the one with short distance
        q_one = selection_joint_angle(q_sol(1:r_idx-1,:), qi);
    end 
end
%% dh_parameters: DH parameter generator
function [dh] = dh_parameters(thetas)
	%% link lengths
	l1 = 50; l2 = 75; l3 = 50; l4 = 20; l5 = 20; l6 = 20;
	%% assigning dh parameters
	a2 = l2; d1 = l1; d4 = l3+l4; d6 = l5+l6;
	dh = [
		0, pi/2, d1, -thetas(1);
		a2, 0, 0, thetas(2)+pi/2;
		0, pi/2, 0, thetas(3);
        0, -pi/2, d4, -thetas(4);
        0, pi/2, 0, thetas(5);
        0, 0, d6, -(thetas(6)+pi/2);
	];
end
%% wrist center
function w_center = wrist_center(thetas)
	%% DH - constants
	dh_const = dh_parameters([thetas(1),thetas(2),thetas(3),0,0,0]);
	a2 = dh_const(2,1); d1 = dh_const(1,3); d4 = dh_const(4,3); 
	%% abbreviations
	c_1 = cos(thetas(1)); s_1 = sin(thetas(1));
	c_2 = cos(thetas(2)); s_2 = sin(thetas(2));
	c_23 = cos(sum(thetas(2:3))); s_23 = sin(sum(thetas(2:3)));
	%% wrist center equations for x, y, z
	w_center = [(-a2*s_2 + d4*c_23)*c_1,(a2*s_2 - d4*c_23)*s_1,(a2*c_2 + d1 + d4*s_23)];
end
%% checking position
function q_sol = position_checking(qs,t_pos)
	%% wrist position calculation
	pos = [
        (wrist_center(qs(1,:)) - t_pos);
        (wrist_center(qs(2,:)) - t_pos);
        (wrist_center(qs(3,:)) - t_pos);
        (wrist_center(qs(4,:)) - t_pos)
    ];
	%% filtering the one with true position
	q_bool = (sum(abs(pos), 2)<1e-5);
	%% returning the one with true position
    q_sol = qs(q_bool,:);
end
%%%% Euler ZYZ orientation
function R = euler_ZYZ(e_angles)
	%% de-structuring
	phi = e_angles(1); theta = e_angles(2); psi = e_angles(3); 
	%% abbreviations
	c_phi = cos(phi); s_phi = sin(phi);
	c_theta = cos(theta); s_theta = sin(theta);
	c_psi = cos(psi); s_psi = sin(psi);
	%% rotational matrix
	R = [
			c_phi*c_theta*c_psi-s_phi*s_psi, -c_phi*c_theta*s_psi-s_phi*c_psi, c_phi*s_theta;
			s_phi*c_theta*c_psi+c_phi*s_psi, -s_phi*c_theta*s_psi+c_phi*c_psi, s_phi*s_theta;
			-s_theta*c_psi, s_theta*s_psi, c_theta;
		];
end
%% using using zyz method
function [q_456_1, q_456_2 ]= zyz_method(R_36)
	%% extracting values from R_36
	r13 = R_36(1,3); r23 = R_36(2,3); r33 =R_36(3,3); 
    r31 = R_36(3,1); r32 = R_36(3,2);
	%% solve for theta five first
	q_5 = [atan2(sqrt(r13^2 + r23^2), r33),atan2(-sqrt(r13^2 + r23^2), r33)];
	if (abs(sin(q_5(1))) > 1e-6)
		q_4 = [atan2(r23, r13), atan2(-r23, -r13)];
		q_6 = [atan2(r32,-r31), atan2(-r32,r31)];
    else
		q_4 = [0,0];
		q_6 = [atan2(-R_36(1,2), R_36(1,1)), atan2(R_36(1,2), -R_36(1,1))];
    end
	%% adding the ones with correct solution
    q_456_1 = []; q_456_2 = [];
	if (sum(abs(R_36 - euler_ZYZ([q_4(1), q_5(1), q_6(1)])),2) < 1e-5)
		q_456_1 = [-q_4(1), q_5(1), q_6(1)-pi/2];
	end
	if (sum(abs(R_36 - euler_ZYZ([q_4(2), q_5(2), q_6(2)])),2) < 1e-5)
		q_456_2 = [q_4(2), q_5(2), q_6(2)-pi/2];
	end
end
%% orientation extraction
function R = orn_extarction(t)
    R = zeros(3);
	R = t(1:3,1:3);
end
%% dh_2_htm: DH to homogeneous transformation matrix converter
function [t] = dh_to_htm(dh)
	t = [
		cos(dh(4)), -sin(dh(4))*cos(dh(2)), sin(dh(4))*sin(dh(2)), dh(1)*cos(dh(4));
		sin(dh(4)), cos(dh(4))*cos(dh(2)), -cos(dh(4))*sin(dh(2)), dh(1)*sin(dh(4));
		0, sin(dh(2)), cos(dh(2)), dh(3);
		0,0,0,1
	];
end
%% forward kinematics
function fk = forward_kinematics(joints)
	fk = eye(4);
	for j=1:size(joints,1)
		fk = fk*dh_to_htm(joints(j,:));
    end
end
%% selection_joint_angle: selecting shortest distance one
function [qf] = selection_joint_angle(q, qi)
	%% selecting the one with minimum
	%% based on angle difference
	[~, index] = min(sum(abs(q - qi),2));
	qf = q(index,:);
end