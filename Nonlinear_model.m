function [xdd] = Nonlinear_model(input)
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
u = input(14);

delta = input(15); 


%% relabeling of constants for nonlinear model(includes cornerining stiffness).
l1 = a1;
l3 = 1.5*b1;
l2 = b1;
b1 = a2;
b2 = input(13)-a1;
b3 = b2; 
b4 = 0.5*b2;



%% state variables
x = input(16);
y = input(17); 
beta = input(18);
theta = input(19);

xd = input(20);
yd = input(21);
beta_d = input(22);
theta_d = input(23); 

%% calculating parameters
SBO = sin(beta-theta); 
CBO = cos(beta-theta);
SDBO = sin(delta-beta-theta) ;
CDBO = cos(delta-beta-theta);
CD = cos(delta) ;
SD = sin(delta) ;

%% calculating forces

x2d = xd + l2*sin(beta)*beta_d + b1*sin(theta)*theta_d;
y2d = yd - l2*cos(beta)*beta_d - b1*cos(theta)*theta_d;


v1 = sqrt(xd^2+yd^2); %% what is v1? 
v2 = sqrt(x2d^2+y2d^2);
beta_1 = atan(yd/xd); % what is beta_1?
beta_2 = atan(y2d/x2d); % what is beta_2?


af1 = delta - beta_d*l1/v1 - beta_1; 
ar1 = beta_d*l3/v1 - beta_1;
af2 = theta_d*b2/v1 - beta_1;
am2 = theta_d*b3/v1 - beta_1;

%Caf1 = C1/100;
%Car1 = C2/100;
%Caf2 = C2/1000;
%Cam2 = C3/300;

Caf1 = C1/100;
Car1 = C2/300;
Caf2 = C2/100;
Cam2 = C3/100;

ar2 = 0;
Fyr2 = 0;

Fyf1 = Caf1*af1;
Fyr1 = Car1*ar1;
Fyf2 = Caf2*af2;
Fym2 = Cam2*am2;


%% calculating Faux

x_dd = input(24);
y_dd = input(25);
beta_dd = input(26);
theta_dd = input(27);
x2_dd = x_dd + l2*sin(beta)*beta_dd + l2*cos(beta)*beta_d^2 + b1*cos(theta)*theta_d^2 + b1*sin(theta)*theta_dd;
y2_dd = y_dd + l2*sin(beta)*beta_dd - l2*cos(beta)*beta_d^2 + b1*sin(theta)*theta_dd - b1*cos(theta)*theta_d^2;


Faux_x = -m2*x2_dd + m1*x_dd;
Faux_y = -m2*y2_dd + m1*y_dd;

Faux_x = m1*x_dd - Fyf1*cos(beta+delta) - Fyr1*cos(beta);
Faux_y = m1*y_dd + Fyf1*sin(beta+delta) + Fyr1*sin(beta);
Faux = sqrt(Faux_x^2 + Faux_y^2);
Faux = 0;
%% lagranian dynamics
M = [(m1+m2),   0,     0,   -b1*m2*SBO;
       0,   (m1+m2), -l2*m2,    -b1*m2*CBO;
       0, -l2*m2, m2*l2^2+ I1, l2*b1*m2*CBO;
       -b1*m2*SBO, -b1*m2*CBO, l2*b1*m2*CBO, m2*b1^2 + I2];
   
   
M_car = [m1,0,0,0;
         0,m1,0,0;
         0,0,I1,0;
         0,0,0,0];
   
C_car = [-beta_d*yd*m1;
          beta_d*xd*m1;
              0;
              0];
                 
Q_car = [Fyf1*SD;
        (-Fyf1*CD - Fyr1);
        (-Fyf1*l1*CD + Fyr1*l3);
        0];

C = [(beta_d^2)*l2*m2+(theta_d^2)*b1*m2*CBO-beta_d*yd*(m1+m2);
     -(beta_d^2)*b1*m2*SBO + beta_d*xd*(m1+m2);
     -l2*m2*(beta_d*xd - (theta_d^2)*b1*SBO);
     -beta_d*b1*m2*(xd*CBO+(beta_d*l2-yd)*SBO)];
     
Q = [(Faux + Fyr2*SDBO + Fyf1*SD - Fyf2*SBO - Fym2*SBO);
     (-Fyr1 - Fyr2*CDBO -Fyf1*CD - Fyf2*CBO - Fym2*CBO);
     (Fyr1*l3 + Fyf2*l2*CBO + Fym2*l2*CBO + Fyr2*l2*CDBO - Fyf1*l1*CD);
     (Fyf2*(b1+b2) + Fym2*(b1+b3)+ Fyr2*CD*(b1+b4))];
     
b = -C+Q;


b_car = -C_car + Q_car;

xdd = M\b;




end

