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
plot(ax1,x,y11,'y',x,y12,'r:','LineWidth',2);
title(ax1,'PositionX');
ylabel(ax1,'cm');
%
ax2 = subplot(6,1,2);
y21 = REF(3,1:end)*100;
y22 = VAL(3,1:end)*100
plot(ax2,x,y21,'y',x,y22,'r:','LineWidth',2);
title(ax2,'PositionY');
ylabel(ax2,'cm');
%
ax3 = subplot(6,1,3);
y31 = REF(4,1:end)*100;
y32 = VAL(4,1:end)*100;
plot(ax3,x,y31,'y',x,y32,'r:','LineWidth',2);
title(ax3,'PositionZ');
ylabel(ax3,'cm');
%
ax4 = subplot(6,1,4);
y41 = REF(5,1:end)*180/pi;
y42 = VAL(5,1:end)*180/pi;
plot(ax4,x,y41,'y',x,y42,'r:','LineWidth',2);
title(ax4,'RotationX');
ylabel(ax4,'deg');
%axis([0 x(1,end) -200 200]);
%
ax5 = subplot(6,1,5);
y51 = REF(6,1:end)*180/pi;
y52 = VAL(6,1:end)*180/pi;
plot(ax5,x,y51,'y',x,y52,'r:','LineWidth',2);
title(ax5,'RotationY');
ylabel(ax5,'deg');
%axis([0 x(1,end) -200 200]);
%
ax6 = subplot(6,1,6);
y61 = REF(7,1:end)*180/pi;
y62 = VAL(7,1:end)*180/pi;
plot(ax6,x,y61,'y',x,y62,'r:','LineWidth',2);
title(ax6,'RotationZ');
ylabel(ax6,'deg');
xlabel(ax6,'seconds');%
%axis([0 x(1,end) -200 200]);
%
fig1 = ax1;
end
%