%% Fcn Kinematics - PLOTTING Exercise of Master Advanced Automation.
%
% function Plotting = F094_PlotTORQUE_7R(torque)
%
% The input "Theta" (8xn) is composed by the following signals:
% 'ti': row with the time for the plot series.
% 'The1': row with the joint_01 torque magnitude Nm.
% 'The2': row with the joint_02 torque magnitude Nm..
% 'The3': row with the joint_03 torque magnitude Nm..
% 'The4': row with the joint_04 torque magnitude Nm..
% 'The5': row with the joint_05 torque magnitude Nm..
% 'The6': row with the joint_06 torque magnitude Nm..
% 'The7': row with the joint_06 torque magnitude Nm..
% 
%
% Use case:
% F094_PlotJOINT_7R('JThe6VAL')
%
% Copyright 2020 Dr. Pardos-Gotor.
%
%% F094_PlotTORQUE_7R
%
function fig1 = F094_PlotTORQUE_7R(torque)
%
JointSeries = load(torque);
JointMat = JointSeries.ans;
% Define the TimeSeries starting point 1sec.
x = JointMat(1,1:end);
%
ax1 = subplot(7,1,1);
y1 = JointMat(2,1:end);
plot(ax1,x,y1,'LineWidth',2);
title(ax1,'Torque1');
xlabel(ax1,'sec');
ylabel(ax1,'Nm');
%
ax2 = subplot(7,1,2);
y2 = JointMat(3,1:end);
plot(ax2,x,y2,'LineWidth',2);
title(ax2,'Torque2');
xlabel(ax2,'sec');
ylabel(ax2,'Nm');
%
ax3 = subplot(7,1,3);
y3 = JointMat(4,1:end);
plot(ax3,x,y3,'LineWidth',2);
title(ax3,'Torque3');
xlabel(ax3,'sec');
ylabel(ax3,'Nm');
%
ax4 = subplot(7,1,4);
y4 = JointMat(5,1:end);
plot(ax4,x,y4,'LineWidth',2);
title(ax4,'Torque4');
xlabel(ax4,'sec');
ylabel(ax4,'Nm');
%
ax5 = subplot(7,1,5);
y5 = JointMat(6,1:end);
plot(ax5,x,y5,'LineWidth',2);
title(ax5,'Torque5');
xlabel(ax5,'sec');
ylabel(ax5,'Nm');
%
ax6 = subplot(7,1,6);
y6 = JointMat(7,1:end);
plot(ax6,x,y6,'LineWidth',2);
title(ax6,'Torque6');
xlabel(ax6,'sec');
ylabel(ax6,'Nm');
%
ax7 = subplot(7,1,7);
y7 = JointMat(8,1:end);
plot(ax7,x,y7,'LineWidth',2);
title(ax7,'Torque7');
xlabel(ax7,'sec');
ylabel(ax7,'Nm');
%
fig1 = ax1;
end
%