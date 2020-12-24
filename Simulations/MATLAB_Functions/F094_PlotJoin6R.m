%% Fcn Kinematics - PLOTTING Exercise of Master Advanced Automation.
%
% function Plotting = F094_PlotJoin6R(tTheta)
%
% The input "Theta" (8xn) is composed by the following signals:
% 'ti': row with the time for the plot series.
% 'The1': row with the joint_01 position magnitude rad series.
% 'The2': row with the joint_02 position magnitude rad series.
% 'The3': row with the joint_03 position magnitude rad series.
% 'The4': row with the joint_04 position magnitude rad series.
% 'The5': row with the joint_05 position magnitude rad series.
% 'The6': row with the joint_06 position magnitude rad series.
% 
%
% Use case:
% F094_PlotJoin6R('JThe6VAL')
%
% Copyright 2020 Dr. Pardos-Gotor.
%
%% F094_PlotJoin6R
%
function fig1 = F094_PlotJoin6R(Join6R)
%
Joints = load(Join6R);
Join = Joints.ans;
% Define the TimeSeries.
x = Join(1,1:end);
%
for i = 1:6
    ax = subplot(6,1,i);
    plot(ax, x, Join(i+1,1:end)*180/pi,'LineWidth',3);
    title(ax,['Theta ', num2str(i)]);
    ylabel(ax,'deg');
end
xlabel(ax,'Seconds');
%
fig1 = ax;
end
%
%