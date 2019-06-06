function [local_xd_x] = GlobtoLoc(input)
%% this function takes the global coordinates from the nonlinear system and
%% converts it to local coordinates to be controlled
local_x = [0 0 0 0];
local_xd = [0 0 0 0];

u = input(14);
x = input(15);
y1 = input(16);
yaw_angle = input(17);
theta = input(18);
xd = input(19); 
y1d = input(20);
yaw_d = input(21);
theta_d = input(22);
xdd = input(23);
y1dd = input(24);
yaw_dd = input(25);
theta_dd = input(26);

%u = constants(14);
%x = global_x(1);
%y1 = global_x(2);
%yaw_angle = global_x(3);
%theta = global_x(4); 
%xd = global_xd(1); 
%y1d = global_xd(2); 
%yaw_d = global_xd(3);
%theta_d = global_xd(4);
%xdd = global_xdd(1);
%y1dd = global_xdd(2);
%yaw_dd = global_xdd(3);
%theta_dd = global_xdd(4);

v1 = y1d - u*yaw_angle;
psi_d = yaw_d;
phi_d = theta_d - yaw_d;
phi = theta - yaw_angle;
local_x = [v1 psi_d phi_d phi]; 


v1d = y1dd - u*yaw_d;
psi_dd = yaw_dd;
phi_dd = theta_dd-yaw_dd;
phi_d = theta_d - yaw_d;
local_xd = [v1d psi_dd phi_dd phi_d];

local_xd_x = [local_xd local_x]';










