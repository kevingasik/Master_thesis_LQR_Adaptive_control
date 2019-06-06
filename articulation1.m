function [A,B1,B2,D] = articulation1(m1,m2,a2,I1,I2,C,C3,Cs1,h1,l2,u,Cq1,C1,a1,l1)

%% Linear And Nonlinear Tractor Trailer Controller ()
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
% this function outputs a mass matrix vector to be used to control for an
% adaptive controller. This function takes 14 arguments
%
%
% *Copyright (C) 2019 Kevin Gasik* This file is not to be used or
% distributed by anyone without explicit consent from the author.This file
% serves as an example of how to approach and simulate these problems and
% should not be used to implement any controllers on any vehicle. 
% By using the program shown below you agree that if anyone is harmed
% Kevin Gasik and California Polytechnic University is not liable. 
b1 = l1-a1;

%% Mass Matrix %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%this is the mass matrix associated with the lagragian derivation for 
%The single articulated vehcile, we make sure to get this 3x3 matrix before
%making a 4x4 matrix
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
%this is the stiffness matrix K that is associated with the tractor-trailer
%model
K = [(C+C3),     (Cs1-C3*(h1+l2)+(m1+m2)*u^2),    (-C3*l2),    (-C3*u); 
     (Cs1-C3*h1),(Cq1^2+C3*h1*(h1+l2)-m2*h1*u^2),  C3*h1*l2,    C3*h1*u; 
     (-C3*l2),   (C3*l2*(h1+l2)-m2*a2*u^2),      (C3*l2^2),    (C3*l2*u);
     0,           0,                              -u,            0];
 
 
%% M inverse Times K %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 MinvK = -1*M_inv*K/u;
 
%% delta and M inverse times delta %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta = [C1;a1*C1;0;0];
MinvDelta = M_inv*delta;

 
%% Error Equations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% we create the the error matrices as detailed in the thesis
% the entire process can be seen by kevin gasik's thesis on adaptive
% control to derive the error states look at chapter 3 and chapter 4.
eddot_A = [1,   0,          0,      0;
           0,   1,          0,      0;
           1,   0,          -a2,     0; 
           0,   1,          1,      0]; 
             
eddot_B = [0,   u,          0,      0;
           0,   0,          0,      0;
           0,   u,          u,      0;
           0,   0,          0,      0];
       
       
Q3 = [-u,0,-u,0]'; % desired yaw rate matrix
%p1 p2 and p3 reprsents the state vector equal to components of the defined
%error equations
p1 = [1,        0,              0,      0;
      0,        1,              0,      0;
      1/a2,     0,           -1/a2,     0;  
      0,        0,              0,      0];
                
p2 = [0,        -u,      0,      0;
      0,         0,      0,      0;
      0,     -u/a2,      0,   u/a2;
      0,        -1,      0,      1]; %% phi = -e2 + e4
               
p3 = [0;1;0;0]; % \dot \psi desired 

 %% Caluclate indivual Matrix Parts
 % we break down this process to make it easier to calculate in the future
 % to add together
Q1 = eddot_A*MinvK;
Q2 = eddot_A*MinvDelta; % delta matrix



C_matrix= Q1 + eddot_B; % matrix where x dot is are the states 
 
X = C_matrix*p1;
Y = C_matrix*p2;

%% Error States augmentation
% here we add the components and aguement the error equations to include
% the lower order terms as well as the higher order terms
E_matrix = zeros(8,8);
E_matrix(1,:) = [0,1,0,0,0,0,0,0];
E_matrix(3,:) = [0,0,0,1,0,0,0,0];
E_matrix(5,:) = [0,0,0,0,0,1,0,0];
E_matrix(7,:) = [0,0,0,0,0,0,0,1];
E_matrix(2,:) = [X(1,1),Y(1,1),X(1,2),Y(1,2),X(1,3),Y(1,3),X(1,4),Y(1,4)]; 
E_matrix(4,:) = [X(2,1),Y(2,1),X(2,2),Y(2,2),X(2,3),Y(2,3),X(2,4),Y(2,4)]; 
E_matrix(6,:) = [X(3,1),Y(3,1),X(3,2),Y(3,2),X(3,3),Y(3,3),X(3,4),Y(3,4)]; 
E_matrix(8,:) = [X(4,1),Y(4,1),X(4,2),Y(4,2),X(4,3),Y(4,3),X(4,4),Y(4,4)];

%Outputs 
A = E_matrix; 

% delta output
B1 = Q2;
B1 = [0,B1(1),0,B1(2),0,B1(3),0,B1(4)]';

%yaw desired output
B2 = Q3 + C_matrix*p3;
B2 = [0,B2(1),0,B2(2),0,B2(3),0,B2(4)]';
 
D = 0;


end

