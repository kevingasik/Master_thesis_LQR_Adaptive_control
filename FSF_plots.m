function [] = FSF_plots(tout,e1,e2,e3,e4,car,hitch,trailer,delta_ang,delta_angle_non,XY_coor)
%% Linear And Nonlinear Tractor Trailer Controller (FSF_Plots)
% This function plots a series of specific variables that the simulink
% models output
%
% *Author:* Kevin Gasik
%
% *Date:*   3/04/2019
%
% *Required Files:* 
% 
% 
% 
%
% *Code Repository:* Visit for source files
%
% 
%
% *Detailed Description:*
% FSF_plots reduces the code in the main script by plotting a series of
% specific variables and displaying them after a successful script run.
%
%
% *Copyright (C) 2019 Kevin Gasik* This file is not to be used or
% distributed by anyone without explicit consent from the author.This file
% serves as an example of how to approach and simulate these problems and
% should not be used to implement any controllers on any vehicle. 
% By using the program shown below you agree that if anyone is harmed
% Kevin Gasik and California Polytechnic University is not liable. 
    hf1 = figure(1);
    hf1.set('Name','Lateral Position Error vs Time',...
        'renderer','painters','position',[360 198 700 420]);
    plot(tout,e1);
    hold on
    plot(tout,e3);
    title({'Lateral Position Error vs Time'},...
       'interpreter','latex','fontsize',14);
    xlabel('Time [$s$]',...
       'interpreter','latex','fontsize',12);
    ylabel('Position [$m$]',...
       'interpreter','latex','fontsize',12);
    legend({'$e_1$','$e_3$'},...
       'interpreter','latex','fontsize',12,'location','bestoutside');

    hf2 = figure(2);
    hf2.set('Name','Yaw Angle Desired vs Time',...
        'renderer','painters','position',[360 198 700 420]);
    plot(tout,e2);
    hold on
    plot(tout,e4);
    title({'Yaw Angle Desired vs Time'},...
       'interpreter','latex','fontsize',14);
    xlabel('Time [$s$]',...
       'interpreter','latex','fontsize',12);
    ylabel('Angle [$radians$]',...
       'interpreter','latex','fontsize',12);
    legend({'$e_2$','$e_4$'},...
       'interpreter','latex','fontsize',12,'location','bestoutside');

    
    hf3 = figure(3);
    hf3.set('Name','Desired Path Global Coordinates',...
        'renderer','painters','position',[360 198 700 420]);
    plot(car(:,1),car(:,2))
    hold on
    plot(trailer(:,1),trailer(:,2))
    plot(XY_coor(:,1),XY_coor(:,2))
    title({'Global Coordinates'},...
       'interpreter','latex','fontsize',14);
    xlabel('Position[$m$]',...
       'interpreter','latex','fontsize',12);
    ylabel('Position [$m$]',...
       'interpreter','latex','fontsize',12);
    legend({'$Tractor$','$Trailer$','$Desired Path$'},...
       'interpreter','latex','fontsize',12,'location','bestoutside');

    hf4 = figure(4);
    hf4.set('Name','Steering Input',...
        'renderer','painters','position',[360 198 700 420]);
    plot(tout,rad2deg(delta_angle_non))
    title({'Steering Input'},...
       'interpreter','latex','fontsize',14);
    xlabel('Time [$s$]',...
       'interpreter','latex','fontsize',12);
    ylabel('Angle [$deg$]',...
       'interpreter','latex','fontsize',12);
    legend({'$steering angle$','$Trailer$','$Path$'},...
       'interpreter','latex','fontsize',12,'location','bestoutside');



    for m = 1:round(length(car(:,1))/50):length(car(:,1))-1
        figure(5)
        plot(car(:,1),car(:,2))
        ax = gca;
        ax.ColorOrderIndex = 1;
        hold on
        plot(trailer(:,1),trailer(:,2))
        plot(XY_coor(:,1),XY_coor(:,2))
        % plot the hitch trailer and car models
        plot([car(m,1),hitch(m,1),trailer(m,1)],[car(m,2),hitch(m,2),trailer(m,2)],'LineWidth',4)
        plot([car(m,1),car(m,1)+5*cos(delta_ang(m,1))],[car(m,2),car(m,2)+5*sin(delta_ang(m,1))],'-')
        plot([hitch(m,1)],[hitch(m,2)],'*')
        plot([car(m,1)],[car(m,2)],'O')
        
        legend('car','trailer','desired')
        xlim([car(m,1)-10,car(m,1)+10])
        ylim([car(m,2)-10,car(m,2)+10])
        %xlim([-2000,2800])
        %ylim([-2000,2800])
        drawnow();
        hold off 
    end 
end

