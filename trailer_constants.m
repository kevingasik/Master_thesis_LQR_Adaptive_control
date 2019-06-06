function [g,m2,a2,l2,b2,e1,k2,I2,Fz3] = trailer_constants(g,m2)
%% Linear And Nonlinear Tractor Trailer Controller (Trailer Constants)
% Assigns the trailer constants for the main script to use.
%
% *Author:* Kevin Gasik
%
% *Date:*   3/04/2019
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
% This function takes two inputs g-gravity and m2-mass of the trailer to
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

g = g;
%%Define trailer constants
m2 = m2; % trailer mass (kg)
a2 = 4.98; % distance from front tire of trailer to trailer center of mass (m)
l2 = 8.13; % distance from front tire to back tire of trailer (m)
b2 = l2 - a2;
e1 = -0.68; % distance rear axel towing point (m)
k2 = 4.05; % raidus of gyration for trailer (m)
I2 = m2*k2^2; % moment of inertia for trailer
Fz3 = a2/l2*m2*g; % Force on the rear wheel of trailer when not moving

end

