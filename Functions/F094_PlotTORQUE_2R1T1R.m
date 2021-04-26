%% Fcn Kinematics - PLOTTING Exercise of Master Advanced Automation.
%
% function Plotting = F094_PlotTORQUE_2R1T1R(torque)
%
% The input "Theta" (8xn) is composed by the following signals:
% 'ti': row with the time for the plot series.
% 'The1': row with the joint_01 torque magnitude Nm.
% 'The2': row with the joint_02 torque magnitude Nm..
% 'The3': row with the joint_03 torque magnitude N.
% 'The4': row with the joint_04 torque magnitude Nm..
% 
%
% Use case:
% F094_PlotTORQUE_2R1T1RR('JTor4VAL')
%
% Copyright 2020 Dr. Pardos-Gotor.
%
%% F094_PlotTORQUE_2R1T1R
%
function fig1 = F094_PlotTORQUE_2R1T1R(torque)
%
JointSeries = load(torque);
JointMat = JointSeries.ans;
% Define the TimeSeries starting point 1sec.
x = JointMat(1,1:end);
%
ax1 = subplot(4,1,1);
y1 = JointMat(2,1:end);
plot(ax1,x,y1,'LineWidth',2);
title(ax1,'Torque1');
xlabel(ax1,'sec');
ylabel(ax1,'Nm');
%
ax2 = subplot(4,1,2);
y2 = JointMat(3,1:end);
plot(ax2,x,y2,'LineWidth',2);
title(ax2,'Torque2');
xlabel(ax2,'sec');
ylabel(ax2,'Nm');
%
ax3 = subplot(4,1,3);
y3 = JointMat(4,1:end);
plot(ax3,x,y3,'LineWidth',2);
title(ax3,'Torque3');
xlabel(ax3,'sec');
ylabel(ax3,'N');
%
ax4 = subplot(4,1,4);
y4 = JointMat(5,1:end);
plot(ax4,x,y4,'LineWidth',2);
title(ax4,'Torque4');
xlabel(ax4,'sec');
ylabel(ax4,'Nm');
%
fig1 = ax1;
end
%