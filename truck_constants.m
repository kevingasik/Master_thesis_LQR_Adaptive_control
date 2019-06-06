function [g,m1,a1,l1,b1,h1,k1,I1,Fz1,Fz2] = truck_constants(g,m1)
%% Linear And Nonlinear Tractor Trailer Controller (Tractor Constants)
% Assigns Tractor Constants in the main script.
%
% *Author:* Kevin Gasik
%
% *Date:*   3/04/2019
% 
% * ITEM1
% * ITEM2
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
% This function takes two inputs g-gravity and m1-mass of the tractor to
% then assign the rest of the trailer constants and output them to the main
% script.
%
%
% *Copyright (C) 2019 Kevin Gasik* This file is not to be used or
% distributed by anyone without explicit consent from the author.This file
% serves as an example of how to approach and simulate these problems and
% should not be used to implement any controllers on any vehicle. 
% By using the program shown below you agree that if anyone is harmed
% Kevin Gasik and California Polytechnic University is not liable. 

g = g; % gravity (m/s^2)

%% Define  truck constants
m1 = 7449; % truck mass (kg)
a1 = 1.10; % distance from front tire of truck to center of mass (m)
l1 = 3.6; % distance from front tire to back tire of truck (m)
b1 = l1 - a1;
h1 = l1 - b1;
k1 = 1.89;%radius of gyration for truck (m)
I1 = m1*k1^2; % moment of inertia for truck
Fz1 = 50620; %front axle load (N)
Fz2 = 22455; %rear axle load (N)

end

