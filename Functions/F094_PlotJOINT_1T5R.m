%% Fcn Kinematics - PLOTTING Exercise of Master Advanced Automation.
%
% function Plotting = F094_PlotJOINT_1T5R(tTheta)
%
% The input "Theta" (8xn) is composed by the following signals:
% 'ti': row with the time for the plot series.
% 'The1': row with the joint_01 position magnitude m series.
% 'The2': row with the joint_02 position magnitude rad series.
% 'The3': row with the joint_03 position magnitude rad series.
% 'The4': row with the joint_04 position magnitude rad series.
% 'The5': row with the joint_05 position magnitude rad series.
% 'The6': row with the joint_06 position magnitude rad series.
% 
%
% Use case:
% F094_PlotJOINT_1T5R('JThe6VAL')
%
% Copyright 2020 Dr. Pardos-Gotor.
%
%% F094_PlotJOINT_1T5R
%
function fig1 = F094_PlotJOINT_1T5R(tTheta)
%
JointSeries = load(tTheta);
JointMat = JointSeries.ans;
% Define the TimeSeries starting point 1sec.
x = JointMat(1,1:end);
%
ax1 = subplot(6,1,1);
y1 = JointMat(2,1:end);
plot(ax1,x,y1,'LineWidth',2);
title(ax1,'Theta 1');
ylabel(ax1,'m');
%
ax2 = subplot(6,1,2);
y2 = JointMat(3,1:end)*180/pi;
plot(ax2,x,y2,'LineWidth',2);
title(ax2,'Theta 2');
ylabel(ax2,'deg');
%
ax3 = subplot(6,1,3);
y3 = JointMat(4,1:end)*180/pi;
plot(ax3,x,y3,'LineWidth',2);
title(ax3,'Theta 3');
ylabel(ax3,'deg');
%
ax4 = subplot(6,1,4);
y4 = JointMat(5,1:end)*180/pi;
plot(ax4,x,y4,'LineWidth',2);
title(ax4,'Theta 4');
ylabel(ax4,'deg');
%
ax5 = subplot(6,1,5);
y5 = JointMat(6,1:end)*180/pi;
plot(ax5,x,y5,'LineWidth',2);
title(ax5,'Theta 5');
ylabel(ax5,'deg');
%
ax6 = subplot(6,1,6);
y6 = JointMat(7,1:end)*180/pi;
plot(ax6,x,y6,'LineWidth',2);
title(ax6,'Theta 6');
ylabel(ax6,'deg');
%
xlabel(ax6,'Seconds');
fig1 = ax1;
end
%