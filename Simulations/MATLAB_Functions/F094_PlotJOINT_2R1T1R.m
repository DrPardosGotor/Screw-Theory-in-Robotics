%% Fcn Kinematics - PLOTTING Exercise of Master Advanced Automation.
%
% function Plotting = F094_PlotJOINT_2R1T1R(tTheta)
%
% The input "Theta" (5xn) is composed by the following signals:
% 'ti': row with the time for the plot series.
% 'The1': row with the joint_01 position magnitude rad series.
% 'The2': row with the joint_02 position magnitude rad series.
% 'The3': row with the joint_03 position magnitude m series.
% 'The4': row with the joint_04 position magnitude rad series.
% 
%
% Use case:
%F094_PlotJOINT_2R1T1R('JThe4VAL')
%
% Copyright 2020 Dr. Pardos-Gotor.
%
%% F094_PlotJOINT_2R1T1R
%
function fig1 = F094_PlotJOINT_2R1T1R(tTheta)
%
JointSeries = load(tTheta);
JointMat = JointSeries.ans;
% Define the TimeSeries starting point 1sec.
x = JointMat(1,1:end);
%
ax1 = subplot(4,1,1);
y1 = JointMat(2,1:end)*180/pi;
plot(ax1,x,y1,'LineWidth',2);
title(ax1,'Theta 1');
ylabel(ax1,'deg');
%
ax2 = subplot(4,1,2);
y2 = JointMat(3,1:end)*180/pi;
plot(ax2,x,y2,'LineWidth',2);
title(ax2,'Theta 2');
ylabel(ax2,'deg');
%
ax3 = subplot(4,1,3);
y3 = JointMat(4,1:end)*100;
plot(ax3,x,y3,'LineWidth',2);
title(ax3,'Theta 3');
ylabel(ax3,'cm');
%
ax4 = subplot(4,1,4);
y4 = JointMat(5,1:end)*180/pi;
plot(ax4,x,y4,'LineWidth',2);
title(ax4,'Theta 4');
ylabel(ax4,'deg');
%
xlabel(ax4,'Seconds');
fig1 = ax1;
end
%