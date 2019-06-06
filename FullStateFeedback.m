function [A_des,B_des,C_des,D_des] = FullStateFeedback(A,B,C,D,Poles)
%% Full State Feedback Controller for Steering Angle
Poles =[(-5-3j) (-5+3j) -7 -10 ];
K = place(A,B1,Poles);
A_prime = (A-B1*K);


%% Transfer function of the system
[b,a] = ss2tf(A_prime,B1,C,D,1);
H = tf(b,a)


%% Transfer Function of desired Poles
syms s
H_des =((s-Poles(1))*(s-Poles(2))*(s-Poles(3))*(s-Poles(4)));
H_des = expand(H_des)
a_des = [1 27 274 1278 2380];
b_des = [0 0 0 1];
[A_des,B_des,C_des,D_des] = tf2ss(b_des,a_des);
end

