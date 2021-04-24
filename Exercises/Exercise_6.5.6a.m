%% Screw Theory - INVERSE DYNAMICS - SVA.
% ABB IRB910SC Home position.
% & Gravity acting in direction -Y (gy).
%
% The goal of this exercise is to prove the INVERSE DYNAMICS
% with T generalized joint torques
% with q generalized joint positions, qd velocities and qdd accelerations.
%
% With the Spatial Vector Algebra.
% RNEA - Recursive Newton-Euler Algorithm by Featherstone
% but with the screw theory POE for the management of the robot kinematics
%
% Copyright (C) 2003-2020, by Dr. Jose M. Pardos-Gotor.
%
% This file is part of The ST24R "Screw Theory Toolbox for Robotics" MATLAB
% 
% ST24R is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published
% by the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% ST24R is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Lesser General Public License
% along with ST24R.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.
%
% CHANGES:
% Revision 1.1  2020/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% E655a_ST24R_ID_ABBIRB910SC_SVA
%
clear;
clc;
%
% Potential Action Vector - Gravity definition (i.e., -g direction).
PoAcc = [0 -9.81 0]';
%
% Degress of Freedon of the Robot
DoF = 4;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mechanical characteristics of the Robot (AT REF HOME POSITION):
% kinematics defined with the screw theory POE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
po=[0;0;0]; pk=[0;0.192;0]; pr=[0.4;0;0]; 
ps=[0.65;0.36;0]; pu=[0.65;0.26;0]; pp=[0.65;0.125;0]; 
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [pk pr ps pu];
Joint = ['rot'; 'rot'; 'tra'; 'rot'];
Axis = [AxisY AxisY AxisY -AxisY];
Twist = zeros(6,DoF);
for i = 1:DoF
    Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvP2tform(pp)*rotX2tform(pi/2)*rotZ2tform(-pi);
%
% Motion RANGE for the robot joints POSITION rad, (by catalog).
Thmax = [pi/180*140 pi/180*150 0 pi/180*400];
Thmin = [-pi/180*140 -pi/180*150 -0.18 -pi/180*400];
% Maximum SPEED for the robot joints m/s and rad/sec, (by catalog).
Thpmax = [7.58 7.58 1.02 pi/180*2400];
Thpmin = -[7.58 7.58 1.02 pi/180*2400];
%
%
% DYNAMIC Parameters of the Robot at REF HOME POSITION - Only aproximation¡
CM = [0.2 0.2 0; 0.5 0.258 0; 0.65 0.258 0; 0.65 0.208 0]';
IT = [0.1 0.3 0.2; 0.1 0.5 0.3; 0.1 0.1 0.1; 0.1 0.1 0.1]';
mass = [7 5 1 0.5];
LiMas = [CM; IT; mass];
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RANDOM definition of POSITION, VELOCITY and ACCELERANTIONS for the
% Joints of the Robot
% TARGET defined by JOINT Position, Velocity & Acceleration
% It is only one random target point and the differentiability of the
% position and velocity trajectory is given for granted. Here we are
% concerned with the Dynamic solution for a single trajectory point.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
Th = zeros(1,DoF); Thp = zeros(1,DoF); Thpp = zeros(1,DoF);
for i = 1:DoF
    Th(i) = rand*Thmax(i)-rand*Thmin(i); % for testing various Theta POS
    Thp(i) = rand*Thpmax(i)-rand*Thpmin(i); % for testing various Theta VEL
    Thpp(i) = rand*Thpmax(i)-rand*Thpmin(i); % for testing various Theta ACC
end
%
% Twist and Magnitude for the Joint position (Th).
TwMag = [Twist; Th];
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ID with the Spatial Vector Algebra.
% by Featherstone Spatial Toolbox 2015.
% RNEA-POE - Recursive Newton-Euler Algorithm with Screw Theory POE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Xts is the transformation from the Tool to the Spatial Systems
% using the Spatial Vector Featherstone nomenclature
% the magnitudes are always measured in the predecesor link
% In fact, Featherstone nomenclature moves the succesor link in order to
% make it coincide wigth the predecessor. In such a way, the X(6x6)
% Featherstone matrix, multiplies magnitudes from the predecesor to give
% as a result the transformed magnitude in the successor frame. 
%
% Matrices of Inertia (I1...I3) defined in LINK Frame with S orientation.
% the postion on the CM is defined in relation to each LINK Frame.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This block of variables is useful for the preparation of the
% first recursive OUTWARDS PASS
%
% Gravitational field is modelled as a fictitious acceleration of the base.
% To do this, we replace the initial value with a0 = - ag = - PoAcc
% With this trick the values for ai and fi (net force body) are no longer
% the true values, as they are offset by gravity acceleration and force.
% Nonetheless, the result of the ID is correct, as we are interested only
% in the Joint Torques.
%
tic;
%
% Gravity definition: PoAcc = [0 0 -9.81]';
ai = [0;0;0; -PoAcc];
%
% Motion Subspace for the Joints.
% Attention, because the third joint is prismatic.
S1 = [Axis(:,1); 0; 0; 0]; S2 = [Axis(:,2); 0; 0; 0];
S3 = [0; 0; 0; Axis(:,3)]; S4 = [Axis(:,4); 0; 0; 0];
S = [S1 S2 S3 S4];
%
% Initial values for the recursive algorithm.
PoE = eye(4); % Product of Exponentials.
Hs0 = eye(4); % Homogeneous transformation for the Link Frame.
Pre = [0; 0; 0]; % Origin of Base Frame.
% Xst stores the Spatial Vector transformation to the base (zero), plus the
% Links Frames and besides the X for the Tool.
Xst = zeros(6,6,DoF+2);
Xst(:,:,1) = eye(6); % the initial zero transformation for the base.
% Link velocity, initial value.
vli = zeros(6,1);
% Body or Link Forces, inital value.
fli = zeros(6,DoF);
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Recursive Algorithm OUTWARDS PASS from Base (Spatial Frame) to Tool
% to calculate Velocities and Accelerations for each Link.
% Besides, the Link forces required are aso calculated.
%
for i = 1:DoF
    % The spatial vector transformation X is obtained with the POE instead
    % of the DH parameters. All Link Coordinate frames have the same
    % orientation of the Spatial frame and are positioned on the points of
    % the Twist for each Joint
    PoE = PoE * ForwardKinematicsPOE(TwMag(:,i));
    Hs0 = Hs0 * trvP2tform(Point(:,i) - Pre);
    Pre = Point(:,i); % Position to the previous Link Frame
    Hsi = PoE * Hs0;
    % Once we have the homogeneous transformation for all Link frames, we
    % store them in form of Spatial Vector Plücker transformation X.
    Xst(:,:,i+1) = tform2xpluc(Hsi);
    % Xst(:,:,i+1) \ Xst(:,:,i) is the X transform spatial MOTION vector
    % between Xi to Xi-1, which is calculated from the FK expressions as
    % Xi_i-1 = inv(X0i) * X0_i-1 = Xi0 * X0_i-1.
    Xi_im1 =  Xst(:,:,i+1) \ Xst(:,:,i);
    % Joint transmitted velocity is obtained with the Joint Motion Subspace
    % & the Joint velocites in the Joint space Thp.
    vji = S(:,i) * Thp(i);
    % Link velocity is obtained with the Joint transmited velocity and the 
    % link velocity of the predecessor link, trasformed to succersor link
    % with spatial vector MOTION transformation X.
    vli = vji + Xi_im1 * vli;
    % Link acceleration with the Featherstone formulation.
    ai =  Xi_im1 * ai + S(:,i) * Thpp(i) + spavec2crm(vli) * vji;
    % fli are the necessary forces applied to the links are calculated
    Ii = LinkInertiaB(CM(:,i)-Point(:,i),diag(IT(:,i)),mass(i));
    fli(:,i) = Ii * ai + spavec2crf(vli) * Ii * vli; 
end
% This last step is not necessary to solve the ID for Joint torques (Force)
% but is useful to complete the FK analysis of the robot
% besides is used for the next implementation of the inwards pass.
% Position of the Tool to the previous Link Frame
Hs0 = Hs0 * trvP2tform(pp - Pre) * rotX2tform(pi/2) * rotZ2tform(-pi);
Hsi = PoE * Hs0;
Xst(:,:,i+2) = tform2xpluc(Hsi);
%
% It is possible to test the FK with both the direct POE and SVA evolution
% from base to tool, thoughout all the Link frames.
%Hst_POE = ForwardKinematicsPOE(TwMag) * Hst0
%Hst_SVA = Hsi
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Recursive Algorithm INWARDS PASS from the Tool to Base (Spatial Frame)
% to calculate Joint Torques (& Forces) required are calculated.
%
ID_SVA = zeros (DoF,1); % 
fji = zeros(6,1); % the Joint in the Tool has zero Force value
%
for i = DoF:-1:1
    % fji is the value of the Joint Force. Input i+1 & output i.
    % fli is the value of the Link Force
    % (Xst(:,:,i+2) \ Xst(:,:,i+1))' is the value of X transform vector
    % between Xi+1 to Xi, which is calculated from the FK expressions as
    % Xi+1_i = inv(X0_i+1) * X0_i = Xi+1_0 * X0i.
    % Attention because the Plücker transformation for forcers works
    % in a dual way to the transformation for motion & velocities.
    % Joint force is obtained with the Link force and the Joint force of 
    % the succesor link, transformed to predecessor link with spatial
    % vector FORCE transformation X.
    % In fact, you can manage transformations without any distinction of 
    % MOTION and FORCE spatial vector, if you follow the logic of the
    % Plücker definition of coordinate transformations.
    % Then, with the TRANSPOSE spatial vector transformation X for motion,
    % it is possible to pass forces from the origin to the end of the X.
    %fji = fli(:,i) + inv(Xst(:,:,i+1) \ Xst(:,:,i+2))' * fji;
    fji = fli(:,i) + (Xst(:,:,i+2) \ Xst(:,:,i+1))' * fji;
    % The Joint Motion Subspace moves the Joint forces to the Joint-Space.
    ID_SVA(i) = S(:,i)' * fji; 
end
ID_SVA
%
toc
%
%