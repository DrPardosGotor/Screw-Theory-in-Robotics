%% Fcn Kinematics - PLOTTING Exercise of Master Advanced Automation.
%
% function Plotting = F091_PlotCARTE(tPxyzRxyz)
%
% The input "tPxyzRxyz" (7xn)is composed by the following signals:
% 'ti': row with the time for the plot series.
% 'Tx': row with X translation of the Goal.
% 'Ty': row with Y translation of the Goal.
% 'Tz': row with Z translation of the Goal.
% 'Rx': row with X rotation of the Goal with Euler (X+Y+Z).
% 'Ry': row with Y rotation of the Goal with Euler (X+Y+Z).
% 'Rz': row with Z rotation of the Goal with Euler (Z+Y+Z).
% 
%
% Use case:
% F091_PlotCARTE('ToolVAL')
%
% Copyright 2020 Dr. Pardos-Gotor.
%
%% F091_PlotCARTE
%
function fig1 = F091_PlotCARTE(tPxyzRxyz)
%
CarteSeries = load(tPxyzRxyz);
CarteMat = CarteSeries.ans;
% Define the TimeSeries starting point 1sec.
x = CarteMat(1,1:end);
%
ax1 = subplot(6,1,1);
y1 = CarteMat(2,1:end)*100;
plot(ax1,x,y1,'y','LineWidth',2);
title(ax1,'Position X');
ylabel(ax1,'cm');
axis([0 x(1,end) 200 400]);
%
ax2 = subplot(6,1,2);
y2 = CarteMat(3,1:end)*100;
plot(ax2,x,y2,'y','LineWidth',2);
title(ax2,'Position Y');
ylabel(ax2,'cm');
axis([0 x(1,end) 100 400]);
%
ax3 = subplot(6,1,3);
y3 = CarteMat(4,1:end)*100;
plot(ax3,x,y3,'y','LineWidth',2);
title(ax3,'Position Z');
ylabel(ax3,'cm');
axis([0 x(1,end) 0 200]);
%
ax4 = subplot(6,1,4);
y4 = CarteMat(5,1:end)*180/pi;
plot(ax4,x,y4,'y','LineWidth',2);
title(ax4,'Orientation X');
ylabel(ax4,'deg');
axis([0 x(1,end) -200 0]);
%
ax5 = subplot(6,1,5);
y5 = CarteMat(6,1:end)*180/pi;
plot(ax5,x,y5,'y','LineWidth',2);
title(ax5,'Orientation Y');
ylabel(ax5,'deg');
axis([0 x(1,end) -50 100]);
%
ax6 = subplot(6,1,6);
y6 = CarteMat(7,1:end)*180/pi;
plot(ax6,x,y6,'y','LineWidth',2);
title(ax6,'Orientation Z');
xlabel(ax6,'seconds');
ylabel(ax6,'deg');
axis([0 x(1,end) -200 200]);
%
fig1 = ax1;
end
%