%% Fcn Kinematics - PLOTTING Exercise of Master Advanced Automation.
%
% function Plotting = F098_PlotJOINT7RefVal(SerREF, SerVAL)
%
% The input "SerREF" and "SerVAL" (7xn) composed by the following signals:
% 'ti': row with the time for the plot series.
% 'The1': row with the joint_01 position magnitude rad series.
% 'The2': row with the joint_02 position magnitude rad series.
% 'The3': row with the joint_03 position magnitude rad series.
% 'The4': row with the joint_04 position magnitude rad series.
% 'The5': row with the joint_05 position magnitude rad series.
% 'The6': row with the joint_06 position magnitude rad series.
% 'The7': row with the joint_07 position magnitude rad series.
% 
%
% Use case:
% F098_PlotJOINT7RefVal('JThe7REF', 'JThe7VAL')
%
% Copyright 2020 Dr. Pardos-Gotor.
%
%% F098_PlotJOINT7RefVal
%
function fig1 = F098_PlotJOINT7RefVal(SerREF, SerVAL)
%
JointREF = load(SerREF);
REF = JointREF.ans;
JointVAL = load(SerVAL);
VAL = JointVAL.ans;
% Define the TimeSeries starting point 1sec.
x = REF(1,1:end);
%
ax1 = subplot(7,1,1);
y11 = REF(2,1:end)*100;
y12 = VAL(2,1:end)*100;
plot(ax1,x,y11,'y',x,y12,'r:','LineWidth',2);
title(ax1,'Theta1');
xlabel(ax1,'sec');
ylabel(ax1,'deg');
%
ax2 = subplot(7,1,2);
y21 = REF(3,1:end)*100;
y22 = VAL(3,1:end)*100
plot(ax2,x,y21,'y',x,y22,'r:','LineWidth',2);
title(ax2,'Theta2');
xlabel(ax2,'sec');
ylabel(ax2,'deg');
%
ax3 = subplot(7,1,3);
y31 = REF(4,1:end)*100;
y32 = VAL(4,1:end)*100;
plot(ax3,x,y31,'y',x,y32,'r:','LineWidth',2);
title(ax3,'Theta3');
xlabel(ax3,'sec');
ylabel(ax3,'deg');
%
ax4 = subplot(7,1,4);
y41 = REF(5,1:end)*180/pi;
y42 = VAL(5,1:end)*180/pi;
plot(ax4,x,y41,'y',x,y42,'r:','LineWidth',2);
title(ax4,'Theta4');
xlabel(ax4,'sec');
ylabel(ax4,'deg');
%
ax5 = subplot(7,1,5);
y51 = REF(6,1:end)*180/pi;
y52 = VAL(6,1:end)*180/pi;
plot(ax5,x,y51,'y',x,y52,'r:','LineWidth',2);
title(ax5,'Theta5');
xlabel(ax5,'sec');
ylabel(ax5,'deg');
%
ax6 = subplot(7,1,6);
y61 = REF(7,1:end)*180/pi;
y62 = VAL(7,1:end)*180/pi;
plot(ax6,x,y61,'y',x,y62,'r:','LineWidth',2);
title(ax6,'Theta6');
xlabel(ax6,'sec');
ylabel(ax6,'deg');
%
ax7 = subplot(7,1,7);
y71 = REF(8,1:end)*180/pi;
y72 = VAL(8,1:end)*180/pi;
plot(ax7,x,y71,'y',x,y72,'r:','LineWidth',2);
title(ax7,'Theta7');
xlabel(ax7,'sec');
ylabel(ax7,'deg');
%
fig1 = ax1;
end
%