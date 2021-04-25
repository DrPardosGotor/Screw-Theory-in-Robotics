%% Fcn Kinematics - PLOTTING Exercise of Master Advanced Automation.
% Plots the trajectory (q,qd,qdd) for ONE PRISMATIC + Five revolute joints.
%
% function Plotting = F096_PlotTraj2R1T1R(Traje2RqT2R)
%
% The input "Traje6R" (19xis composed by the following signals:
% row 1 is the trajectory line time.
% rows 2-5 is Joint Theta Positon q (rad).
% rows 6-9 is Joint Theta Velocity qd (rad/s).
% rows 10-13 is Joint Theta Acceleration qdd (rad/ss).
% 
%
% Use case:
% F096_PlotTraj1T5R('Traje2RqT2R')
%
% Copyright 2020 Dr. Pardos-Gotor.
%
%% F096_PlotTraj2R1T1R
%
function fig1 = F096_PlotTraj2R1T1R(Traje2RqT2R)
%
Trajectory = load(Traje2RqT2R);
Tra = Trajectory.ans;
% Define the TimeSeries starting point 1sec.
x = Tra(1,1:end);
%
for i = 1:2
    ax = subplot(4,1,i);
    title(ax,['Joint ', num2str(i)]);
    yyaxis left;
    plot(ax, x, Tra(i+1,1:end),'LineWidth',3); hold on;
    plot(ax, x, Tra(i+5,1:end),'-r','LineWidth',2); hold on;
    ylabel(ax,'rad \color{black} - \color{red} rad/s');
    yyaxis right;
    plot(ax, x, Tra(i+9,1:end),'LineWidth',1); hold off;
    ylabel(ax,'rad/ss');
end
%
    ax = subplot(4,1,3);
    title(ax,'Joint 3');
    yyaxis left;
    plot(ax, x, Tra(4,1:end)*100,'LineWidth',3); hold on;
    plot(ax, x, Tra(8,1:end)*100,'-r','LineWidth',2); hold on;
    ylabel(ax,'cm \color{black} - \color{red} cm/s');
    yyaxis right;
    plot(ax, x, Tra(12,1:end)*100,'LineWidth',1); hold off;
    ylabel(ax,'cm/ss');
    %
    ax = subplot(4,1,4);
    title(ax,'Joint 4');
    yyaxis left;
    plot(ax, x, Tra(5,1:end),'LineWidth',3); hold on;
    plot(ax, x, Tra(9,1:end),'-r','LineWidth',2); hold on;
    ylabel(ax,'rad \color{black} - \color{red} rad/s');
    yyaxis right;
    plot(ax, x, Tra(13,1:end),'LineWidth',1); hold off;
    ylabel(ax,'rad/ss');
    %
xlabel(ax,'Seconds');
fig1 = ax;
end
%