%% Fcn Kinematics - PLOTTING Exercise of Master Advanced Automation.
%
% function Plotting = F092_PlotCARTERefVal(SerREF, SerVAL)
%
% The input "SerREF" and "SerVAL" (7xn) composed by the following signals:
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
% F092_PlotCARTERefVal('ToolREF', 'ToolVAL')
%
% Copyright 2020 Dr. Pardos-Gotor.
%
%% F092_PlotCARTERefVal
%
function fig1 = F092_PlotCARTERefVal(SerREF, SerVAL)
%
CarteREF = load(SerREF);
REF = CarteREF.ans;
CarteVAL = load(SerVAL);
VAL = CarteVAL.ans;
% Define the TimeSeries starting point 1sec.
x = REF(1,1:end);
%
ax1 = subplot(6,1,1);
y11 = REF(2,1:end)*100;
y12 = VAL(2,1:end)*100;
plot(ax1,x,y11,'b',x,y12,'r:','LineWidth',5);
title(ax1,'Position X');
ylabel(ax1,'cm');
set(gca,'FontSize',30);
%axis([0 x(1,end) -20 40]);
%
ax2 = subplot(6,1,2);
y21 = REF(3,1:end)*100;
y22 = VAL(3,1:end)*100;
plot(ax2,x,y21,'b',x,y22,'r:','LineWidth',5);
title(ax2,'Position Y');
ylabel(ax2,'cm');
set(gca,'FontSize',30);
%axis([0 x(1,end) -20 20]);
%
ax3 = subplot(6,1,3);
y31 = REF(4,1:end)*100;
y32 = VAL(4,1:end)*100;
plot(ax3,x,y31,'b',x,y32,'r:','LineWidth',5);
title(ax3,'Position Z');
ylabel(ax3,'cm');
set(gca,'FontSize',30);
%axis([0 x(1,end) 125 140]);
%
ax4 = subplot(6,1,4);
y41 = REF(5,1:end)*180/pi;
y42 = VAL(5,1:end)*180/pi;
plot(ax4,x,y41,'b',x,y42,'r:','LineWidth',5);
title(ax4,'Orientation X');
ylabel(ax4,'deg');
set(gca,'FontSize',30);
%axis([0 x(1,end) 89 91]);
%
ax5 = subplot(6,1,5);
y51 = REF(6,1:end)*180/pi;
y52 = VAL(6,1:end)*180/pi;
plot(ax5,x,y51,'b',x,y52,'r:','LineWidth',5);
title(ax5,'Orientation Y');
ylabel(ax5,'deg');
set(gca,'FontSize',30);
%axis([0 x(1,end) -20 40]);
%
ax6 = subplot(6,1,6);
y61 = REF(7,1:end)*180/pi;
y62 = VAL(7,1:end)*180/pi;
plot(ax6,x,y61,'b',x,y62,'r:','LineWidth',5);
title(ax6,'Orientation Z');
ylabel(ax6,'deg');
set(gca,'FontSize',30);
%
xlabel(ax6,'Seconds','FontSize',40);
%axis([0 x(1,end) -200 200]);
%
fig1 = ax1;
legend('TARGET Tool Trajectory','ACTUAL Tool Trajectory','FontSize',30)
end
%