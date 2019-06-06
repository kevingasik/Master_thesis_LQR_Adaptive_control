function [Ap,Bp,Cp,Dp] = articulation(m1,m2,a2,I1,I2,C,C3,Cs1,h1,l2,u,Cq1,C1,a1,l1)
%% Linear And Nonlinear Tractor Trailer Controller (Main Script)
% Simulates a tractor-trailer model
%
% *Author:* Kevin Gasik
%
% *Date:*   3/04/2019
%
% *Required Files:* (click for documentation)
% 
% 
% 
% *See Also:*
%
% * 
%
% *Code Repository:* Visit for source files
%
% 
%
% *Detailed Description:*

%
%
% *Copyright (C) 2019 Kevin Gasik* This file is not to be used or
% distributed by anyone without explicit consent from the author.This file
% serves as an example of how to approach and simulate these problems and
% should not be used to implement any controllers on any vehicle. 
% By using the program shown below you agree that if anyone is harmed
% Kevin Gasik and California Polytechnic University is not liable. 

Ap = eye(4)
Bp = [0;0;0;0];
Cp = [0 0 0 0]
Dp = 0

%% Mass Matrix %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = [(m1+m2), -m2*(h1+a2), -m2*a2; 
    -m2*h1, (I1+m2*h1*(h1+a2)), m2*h1*a2; 
    -m2*a2, (I2+m2*a2*(h1+a2)), I2+m2*a2^2];
%this is the mass matrix associated with the lagragian derivation for 
%The single articulated vehcile
mass_matrix = [M(1,1), M(1,2),M(1,3), 0;
               M(2,1), M(2,2),M(2,3), 0;
               M(3,1), M(3,2),M(3,3), 0;
               0,      0,     0,      1];

M_inv = inv(mass_matrix); 
               
%% Stiffness K Matrix 
K = [(C+C3),     (Cs1-C3*(h1+l2)+(m1+m2)*u^2),    (-C3*l2),    (-C3*u); 
     (Cs1-C3*h1),(Cq1^2+C3*h1*(h1+l2)-m2*h1*u^2),  C3*h1*l2,    C3*h1*u; 
     (-C3*l2),   (C3*l2*(h1+l2)-m2*a2*u^2),      (C3*l2^2),    (C3*l2*u);
     0,           0,                              -u,            0];
 
 
%% M inverse Times K %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ap = -1*M_inv*K/u;
 
%% delta and M inverse times delta %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta = [C1;a1*C1;0;0];
Bp = M_inv*delta;
Cp = [0 1 0 0];

end

