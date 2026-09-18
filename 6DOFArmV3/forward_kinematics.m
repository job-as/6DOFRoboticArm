%% forward kinematics
function [pos, orn] = forward_kinematics(thetas)
    %% dh parameters
    joints = dh_parameters(thetas);
    %% temporary variable
	fk = eye(4);
	for j=1:size(joints,1)
		fk = fk*dh_to_htm(joints(j,:));
    end
    %% extracting position
    pos = pos_extraction(fk);
    %% orientation extraction
    orn = orn_extraction(fk);

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
%% orientation extraction
function P = pos_extraction(t)
	P = t(1:3,4);
end
%% orientation extraction
function R = orn_extraction(t)
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