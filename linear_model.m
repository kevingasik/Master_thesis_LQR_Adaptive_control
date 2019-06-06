function [xdd] = linear_model(input)
xdd = [0;0;0;0];

%% constants
C1 = input(1);
C2 = input(2);
C3 = input(3);
m1 = input(4);
m2 = input(5);
I1 = input(6);
I2 = input(7);
a1 = input(8);
l1 = input(9);
h1 = input(10);
b1 = input(11);
a2 = input(12);
l2 = input(13);
u =  input(14);

C = C1 + C2;
Cs1 = a1*C1 - b1*C2;
Cq1 = sqrt(a1^2*C1 +b1^2*C2);

steer = input(15); 

%%state variables
x_bar = [input(20);input(21);input(22);input(23)];
%x_bar = [input(16);input(17);input(18);input(19)];

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

              
%% Stiffness K Matrix 
K = [(C+C3),     (Cs1-C3*(h1+l2)+(m1+m2)*u^2),    (-C3*l2),    (-C3*u); 
     (Cs1-C3*h1),(Cq1^2+C3*h1*(h1+l2)-m2*h1*u^2),  C3*h1*l2,    C3*h1*u; 
     (-C3*l2),   (C3*l2*(h1+l2)-m2*a2*u^2),      (C3*l2^2),    (C3*l2*u);
     0,           0,                              -u,            0];
 
 
%% M inverse Times K %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MinvK = -1/u*mass_matrix \ K*x_bar;
K = -1/u*K*x_bar;
%% delta and M inverse times delta %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta = [C1;a1*C1;0;0];
D = delta*steer;

%MinvDelta = mass_matrix \ delta;

xdd = mass_matrix \ (K) + mass_matrix \ D;
