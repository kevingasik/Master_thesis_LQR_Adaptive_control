function [Fn,C1,C2,C,C3,Cs1,Cq1] = model_constants(Fz1,Fz3,a1,b1)
%% Linear And Nonlinear Tractor Trailer Controller (model constants)
% Assigns Tractor Constants in the main script.
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
% This function takes four inputs to calculate the models coefficient of
% coerning stiffnesses
%
%
% *Copyright (C) 2019 Kevin Gasik* This file is not to be used or
% distributed by anyone without explicit consent from the author.This file
% serves as an example of how to approach and simulate these problems and
% should not be used to implement any controllers on any vehicle. 
% By using the program shown below you agree that if anyone is harmed
% Kevin Gasik and California Polytechnic University is not liable. 

%%Define Model Constants
Fn = 5.73; %normalized cornerning stifness
C1 = Fz1*Fn; % cornering stiffness on the front wheel of truck
C2 = Fz1*Fn; % cornering stiffness on the rear wheel of truck
C = C1 + C2; % simplication for cornering stiffness
C3 = Fz3*Fn; % Cornering stiffness on rear wheel of trailer
Cs1 = a1*C1 - b1*C2;
Cq1 = sqrt(a1^2*C1 +b1^2*C2);

end

