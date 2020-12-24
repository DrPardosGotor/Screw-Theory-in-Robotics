%% Fcn Kinematics - PLOTTING Exercise of Master Advanced Automation.
%
% function Plotting = F093_PlotCARTE3D(S1xyz, S2xyz)
%
% The inputs "u" are composed by the following Signals Sxyz.
% 'S1xyz' (3xn): File with TARGET translation of the desired Goal.
% 'S2xyz' (3xn): File with TCP translation.
%
% Use case
% F093_PlotCARTE3D('ToolREF', 'ToolVAL')
% ToolREF = Target Tool trajectory (POS + ORI), matlab file (8xn)
% ToolVAL = Tool TCP trajectory (POS + ORI), matlab file (7xn)
% the function works taking only the files (3xn) POSITION X-Y-Z DATA
% included in the rows 2-4 of the mat files.
%
% Copyright 2020 Dr. Pardos-Gotor.
%
%% F093_PlotCARTE3D
%
function fig1 = F093_PlotCARTE3D(S1, S2)
%
Target = load(S1);
ToolRefXYZ = Target.ans;
TCP = load(S2);
ToolValXYZ = TCP.ans;
%
fig1 = plot3([0, 0, 0,],[0.01, 0.01, 0.01,],[0.01, 0.01, 0.01,],'o','LineWidth', 3,'DisplayName','Sorigin');
%
%xlim([0 0.75]);
%ylim([-0.5 0.5]);
%zlim([0 0.75]);
%
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
hold on;
key = input('plot TCPxyz REFERENCE?','s');
if (key == 'Y')||(key == 'y')
    plot3(ToolRefXYZ(2,:), ToolRefXYZ(3,:), ToolRefXYZ(4,:),'Color','y','LineWidth', 3,'DisplayName','TARGET-REFxyz');
end
%
key = input('plot TCPxyz VALUE?','s');
if (key == 'Y')||(key == 'y')
    plot3(ToolValXYZ(2,:), ToolValXYZ(3,:), ToolValXYZ(4,:),'Color','r','LineWidth', 3,'DisplayName','TCP-VALxyz');
end
hold off;
%
end
%