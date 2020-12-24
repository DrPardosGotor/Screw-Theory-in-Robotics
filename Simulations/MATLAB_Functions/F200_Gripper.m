%% Fcn Kinematics GRIPPER for a KUKA IIWA Man.
%
% function Gripper = F200_Gripper(u)
%
% The inputs "u" are composed by the following vectors.
% "PosNOAP" (3x1) translation of the desired Goal (noap - "p" goal).
% "RotNOAP" (3x1) rotations or the desired Goal (noap - "noa" order X+Y+Z).
% "PosTCP" (3x1) translation of the robot TcP ("p").
% "RotTCP" (3x1) rotations or the robot TcP (order X+Y+Z).
%
% "Gripper" (1x1) value "1" if NOAP & TCP coincide. If the difference
% between Translation & Orientation (TcP-NOAP) is lower than 0.01 (this is
% basically coincede, then Gripper = 150 (this means the gripper closes),
% otherwise Gripper = 100 (this means the gripper opens).
%
% Copyright 2017-2020 Dr. Pardos-Gotor.
%
%% F200_Gripper
%
function Gripper = F200_Gripper(u)
%
    if sum(abs(u(1:6)-u(7:12))) < 0.03
        Gripper = 150;
    else
        Gripper = 100;
    end
%
end
%