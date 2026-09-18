% Simscape(TM) Multibody(TM) version: 24.2

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(13).translation = [0.0 0.0 0.0];
smiData.RigidTransform(13).angle = 0.0;
smiData.RigidTransform(13).axis = [0.0 0.0 0.0];
smiData.RigidTransform(13).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [0 0 10];  % mm
smiData.RigidTransform(1).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(1).axis = [-0.70710678118654746 0.70710678118654757 0];
smiData.RigidTransform(1).ID = "B[base:1:-:link1:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [0 0 3.0000000000000004];  % mm
smiData.RigidTransform(2).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(2).axis = [-0.70710678118654735 0.70710678118654768 0];
smiData.RigidTransform(2).ID = "F[base:1:-:link1:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [0 1.9999999999999996 53.000000000000007];  % mm
smiData.RigidTransform(3).angle = 1.5707963267948968;  % rad
smiData.RigidTransform(3).axis = [1 0 0];
smiData.RigidTransform(3).ID = "B[link1:1:-:link2:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [-1.1102230246251565e-15 2.0000000000000018 5];  % mm
smiData.RigidTransform(4).angle = 1.5707963267948968;  % rad
smiData.RigidTransform(4).axis = [1 0 0];
smiData.RigidTransform(4).ID = "F[link1:1:-:link2:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [1.1102230246251565e-15 1.9999999999999996 80];  % mm
smiData.RigidTransform(5).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(5).axis = [0 -0.70710678118654746 0.70710678118654757];
smiData.RigidTransform(5).ID = "B[link2:1:-:link3:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [-2.2204460492503131e-15 2.0000000000000009 5];  % mm
smiData.RigidTransform(6).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(6).axis = [0.57735026918962584 0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(6).ID = "F[link2:1:-:link3:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [0 0 55];  % mm
smiData.RigidTransform(7).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(7).axis = [1 0 0];
smiData.RigidTransform(7).ID = "B[link3:1:-:link4:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [0 0 0];  % mm
smiData.RigidTransform(8).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(8).axis = [1 1.1102230246251568e-16 0];
smiData.RigidTransform(8).ID = "F[link3:1:-:link4:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(9).translation = [1.1102230246251565e-15 1.9999999999999996 20];  % mm
smiData.RigidTransform(9).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(9).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(9).ID = "B[link4:1:-:link5:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(10).translation = [0 1.9999999999999996 5];  % mm
smiData.RigidTransform(10).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(10).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(10).ID = "F[link4:1:-:link5:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(11).translation = [0 0 25];  % mm
smiData.RigidTransform(11).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(11).axis = [1 -1.1102230246251565e-16 0];
smiData.RigidTransform(11).ID = "B[link5:1:-:link6:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(12).translation = [0 0 0];  % mm
smiData.RigidTransform(12).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(12).axis = [-0.70710678118654746 0.70710678118654757 0];
smiData.RigidTransform(12).ID = "F[link5:1:-:link6:1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(13).translation = [-8.2036770555204654 -4.7029702269499847 -5.0000000000000009];  % mm
smiData.RigidTransform(13).angle = 0;  % rad
smiData.RigidTransform(13).axis = [0 0 0];
smiData.RigidTransform(13).ID = "RootGround[base:1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(7).mass = 0.0;
smiData.Solid(7).CoM = [0.0 0.0 0.0];
smiData.Solid(7).MoI = [0.0 0.0 0.0];
smiData.Solid(7).PoI = [0.0 0.0 0.0];
smiData.Solid(7).color = [0.0 0.0 0.0];
smiData.Solid(7).opacity = 0.0;
smiData.Solid(7).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.021026173288368734;  % kg
smiData.Solid(1).CoM = [4.967634821534702e-11 0 4.767829398951414];  % mm
smiData.Solid(1).MoI = [3.406045968863828 4.3631578781393596 7.4319168280692276];  % kg*mm^2
smiData.Solid(1).PoI = [0 -3.900298850756365e-12 0];  % kg*mm^2
smiData.Solid(1).color = [0.92156862745098034 0.92156862745098034 0.92156862745098034];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "base.ipt_{34D6C749-4C74-2D56-12BE-42888E692386}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.0063353531489845774;  % kg
smiData.Solid(2).CoM = [3.3711335015770666e-10 -0.16483124231088048 22.626308154875041];  % mm
smiData.Solid(2).MoI = [1.9494404474254032 1.9508838603455563 0.15775760975884529];  % kg*mm^2
smiData.Solid(2).PoI = [0.029551694028413549 4.2729147057194902e-11 -7.9385920756478234e-14];  % kg*mm^2
smiData.Solid(2).color = [0.92156862745098034 0.92156862745098034 0.92156862745098034];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "link1.ipt_{C14F885E-4877-90D5-0BDA-B3A148D41810}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 0.0069810169765974033;  % kg
smiData.Solid(3).CoM = [0 0 44.7392788611782];  % mm
smiData.Solid(3).MoI = [3.1998997311189612 3.1991544126950515 0.10204528682201251];  % kg*mm^2
smiData.Solid(3).PoI = [0.073986884028115332 -1.4985859341401586e-12 0];  % kg*mm^2
smiData.Solid(3).color = [0.92156862745098034 0.92156862745098034 0.92156862745098034];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "link2.ipt_{FA7F61ED-42A8-AA47-4589-8DB266E16F96}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 0.004479943412239288;  % kg
smiData.Solid(4).CoM = [-5.5112365331625224e-12 0.23309761618246877 31.665401960348483];  % mm
smiData.Solid(4).MoI = [0.87923527994126049 0.87746208497335587 0.06671678239812065];  % kg*mm^2
smiData.Solid(4).PoI = [0.025679259918006965 -4.281097044820359e-13 0];  % kg*mm^2
smiData.Solid(4).color = [0.92156862745098034 0.92156862745098034 0.92156862745098034];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = "link3.ipt_{F938E8AC-4481-8BEE-8D3F-FC8A820A0B05}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(5).mass = 0.0020120195947818786;  % kg
smiData.Solid(5).CoM = [7.5937963527076309e-12 -0.51901290262460587 10.784986610447213];  % mm
smiData.Solid(5).MoI = [0.10669934562225376 0.10851262420285429 0.027270994434250594];  % kg*mm^2
smiData.Solid(5).PoI = [0.0074564450785210793 -2.3950800228295948e-13 0];  % kg*mm^2
smiData.Solid(5).color = [0.92156862745098034 0.92156862745098034 0.92156862745098034];
smiData.Solid(5).opacity = 1;
smiData.Solid(5).ID = "link4.ipt_{674B41AC-4FF0-B1E1-071E-749A4EC5ADBB}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(6).mass = 0.0015456195947818872;  % kg
smiData.Solid(6).CoM = [-1.5968103142654555e-11 0.6756281646213812 16.133218162340793];  % mm
smiData.Solid(6).MoI = [0.066764150034087963 0.065453073844359194 0.022622015901727993];  % kg*mm^2
smiData.Solid(6).PoI = [0.0094595575167869018 -1.8547630362536235e-13 -5.8387883750727874e-14];  % kg*mm^2
smiData.Solid(6).color = [0.92156862745098034 0.92156862745098034 0.92156862745098034];
smiData.Solid(6).opacity = 1;
smiData.Solid(6).ID = "link5.ipt_{F57403D2-4665-89CD-B944-3B91E474346A}";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(7).mass = 0.0019562158783049326;  % kg
smiData.Solid(7).CoM = [-4.8054866112699784e-12 0 9.8467919913766622];  % mm
smiData.Solid(7).MoI = [0.080304910071349003 0.080304910071349003 0.029088431812144995];  % kg*mm^2
smiData.Solid(7).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(7).color = [0.92156862745098034 0.92156862745098034 0.92156862745098034];
smiData.Solid(7).opacity = 1;
smiData.Solid(7).ID = "link6.ipt_{BC9F3E1A-4258-CA89-8050-829E38F05595}";


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(6).Rz.Pos = 0.0;
smiData.RevoluteJoint(6).ID = "";

smiData.RevoluteJoint(1).Rz.Pos = -2.5444437451708131e-14;  % deg
smiData.RevoluteJoint(1).ID = "[base:1:-:link1:1]";

smiData.RevoluteJoint(2).Rz.Pos = 0;  % deg
smiData.RevoluteJoint(2).ID = "[link1:1:-:link2:1]";

smiData.RevoluteJoint(3).Rz.Pos = -1.5605378319924397e-14;  % deg
smiData.RevoluteJoint(3).ID = "[link2:1:-:link3:1]";

smiData.RevoluteJoint(4).Rz.Pos = -3.1805546814635164e-15;  % deg
smiData.RevoluteJoint(4).ID = "[link3:1:-:link4:1]";

smiData.RevoluteJoint(5).Rz.Pos = 0;  % deg
smiData.RevoluteJoint(5).ID = "[link4:1:-:link5:1]";

smiData.RevoluteJoint(6).Rz.Pos = -3.0875225029818259e-14;  % deg
smiData.RevoluteJoint(6).ID = "[link5:1:-:link6:1]";

