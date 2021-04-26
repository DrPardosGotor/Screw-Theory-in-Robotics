%% Fcn Kinematics - PLOTTING Exercise of Master Advanced Automation.
% Plots the trajectory (q,qd,qdd) for SIX revolute joints.
%
% function Plotting = F096_PlotTraj6R(Traje6R)
%
% The input "Traje6R" (19xis composed by the following signals:
% row 1 is the trajectory line time.
% rows 2-7 is Joint Theta Positon q (rad).
% rows 8-13 is Joint Theta Velocity qd (rad/s).
% rows 14-19 is Joint Theta Acceleration qdd (rad/ss).
% 
%
% Use case:
% F096_PlotTraj6R(Traje6RTrape)
%
% Copyright 2020 Dr. Pardos-Gotor.
%
%% F096_PlotTraj6R
%
function fig1 = F096_PlotTraj6R(Traje6R)
%
Trajectory = load(Traje6R);
Tra = Trajectory.ans;
% Define the TimeSeries starting point 1sec.
x = Tra(1,1:end);
%
for i = 1:6
    ax = subplot(6,1,i);
    title(ax,['Joint ', num2str(i)]);
    yyaxis left;
    plot(ax, x, Tra(i+1,1:end),'-b','LineWidth',5); hold on;
    plot(ax, x, Tra(i+7,1:end),'-.r','LineWidth',5); hold on;
    ylabel(ax,'\color{blue} rad - \color{red} rad/s');
    yyaxis right;
    plot(ax, x, Tra(i+13,1:end),':m','LineWidth',5); hold off;
    ylabel(ax,'\color{magenta} rad/ss');
    set(gca,'FontSize',30);
end
xlabel(ax,'Seconds','FontSize',30);
legend('Magnitude','Velocity','Acceleration','FontSize',30)
%
fig1 = ax;
end
%