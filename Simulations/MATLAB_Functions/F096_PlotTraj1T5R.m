%% Fcn Kinematics - PLOTTING Exercise of Master Advanced Automation.
% Plots the trajectory (q,qd,qdd) for ONE PRISMATIC + Five revolute joints.
%
% function Plotting = F096_PlotTraj1T5R(Traje6R)
%
% The input "Traje6R" (19xis composed by the following signals:
% row 1 is the trajectory line time.
% rows 2-7 is Joint Theta Positon q (rad).
% rows 8-13 is Joint Theta Velocity qd (rad/s).
% rows 14-19 is Joint Theta Acceleration qdd (rad/ss).
% 
%
% Use case:
% F096_PlotTraj1T5R('Traje1T5RTrape')
%
% Copyright 2020 Dr. Pardos-Gotor.
%
%% F096_PlotTraj1T5R
%
function fig1 = F096_PlotTraj1T5R(Traje6R)
%
Trajectory = load(Traje6R);
Tra = Trajectory.ans;
% Define the TimeSeries starting point 1sec.
x = Tra(1,1:end);
%
    ax = subplot(6,1,1);
    title(ax,'Joint 1');
    yyaxis left;
    plot(ax, x, Tra(2,1:end),'LineWidth',3); hold on;
    plot(ax, x, Tra(8,1:end),'-r','LineWidth',2); hold on;
    ylabel(ax,'m \color{black} - \color{red} m/s');
    yyaxis right;
    plot(ax, x, Tra(14,1:end),'LineWidth',1); hold off;
    ylabel(ax,'m/ss');
    %
for i = 2:6
    ax = subplot(6,1,i);
    title(ax,['Joint ', num2str(i)]);
    yyaxis left;
    plot(ax, x, Tra(i+1,1:end),'LineWidth',3); hold on;
    plot(ax, x, Tra(i+7,1:end),'-r','LineWidth',2); hold on;
    ylabel(ax,'rad \color{black} - \color{red} rad/s');
    yyaxis right;
    plot(ax, x, Tra(i+13,1:end),'LineWidth',1); hold off;
    ylabel(ax,'rad/ss');
end
xlabel(ax,'Seconds');
%
fig1 = ax;
end
%