function [A_p,B1_p,B2_p,D_p] = FSF_integral(A,B1,B2,D)
%% This function takes a state space matrix and adds an integral state for
% the use of full state feedback with integral control
vector = [0;0;0;0;0;0;0;0;-1];
A_p = [A;1 0 0 0 1 0 0 0];
A_p = [A_p vector];
B1_p = [B1;0];
B2_p = [B2;0];
D_p = [D;0];

end

