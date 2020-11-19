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
plot(ax1,x,y1,'LineWidth',2);
title(ax1,'PosX');
xlabel(ax1,'sec');
ylabel(ax1,'cm');
%
ax2 = subplot(6,1,2);
y2 = CarteMat(3,1:end)*100;
plot(ax2,x,y2,'LineWidth',2);
title(ax2,'PosY');
xlabel(ax2,'sec');
ylabel(ax2,'cm');
%
ax3 = subplot(6,1,3);
y3 = CarteMat(4,1:end)*100;
plot(ax3,x,y3,'LineWidth',2);
title(ax3,'PosZ');
xlabel(ax3,'sec');
ylabel(ax3,'cm');
%
ax4 = subplot(6,1,4);
y4 = CarteMat(5,1:end)*180/pi;
plot(ax4,x,y4,'LineWidth',2);
title(ax4,'RotX');
xlabel(ax4,'sec');
ylabel(ax4,'deg');
axis([0 x(1,end) -200 200]);
%
ax5 = subplot(6,1,5);
y5 = CarteMat(6,1:end)*180/pi;
plot(ax5,x,y5,'LineWidth',2);
title(ax5,'RotY');
xlabel(ax5,'sec');
ylabel(ax5,'deg');
axis([0 x(1,end) -200 200]);
%
ax6 = subplot(6,1,6);
y6 = CarteMat(7,1:end)*180/pi;
plot(ax6,x,y6,'LineWidth',2);
title(ax6,'RotZ');
xlabel(ax6,'sec');
ylabel(ax6,'deg');
axis([0 x(1,end) -200 200]);
%
fig1 = ax1;
end
%