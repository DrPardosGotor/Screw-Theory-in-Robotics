%% FORWARDKINEMATICSDHC Forward Kinematics Rigid Body Tree by DH parameters
% DHC for Denavit-Hartenberg CLASSIC algorithm.
% Calculation of the D-H Joint Matrix with ST24R.
%
% Hst = ForwardKinematicsDHC(dhcparams)
%
% Forward kinematics compute end-effector motion (position & orientation)
% in terms of given joints motion defined by its DH parameters.
%
%   H = FORWARDKINEMATICSDHC(DHCPARAMS) gets the homogeneous matrix H for 
%   the end-effector of the robot (or rigid body tree of n links)
%   from the Denavit-Hartenberg classic parameters of each link.
%   dhparams = [d1 t1 r1 a1; d2 t2 r2 a2;...;dn tn rn an] (nx4)
%   for each rigid body (link 1..n).
%       d: The translation on Z(i-1) axis (length units).
%       t: The rotation about Z(i-1) axis (radians). 
%       r: The length on Xi axis of the common normal (joint offset) in 
%          (length units).
%       a: The angle on Xi axis between successive Z(i-1) & Zi axes 
%          (joint twist) in (radians).
%   Only d and t are joint variables.
%
%
% See also BodyDHC2tform(dhparams)
%
% Copyright (C) 2001-2020, by Dr. Jose M. Pardos-Gotor.
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
% You should have received a copy of the GNU Leser General Public License
% along with ST24R.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.
%
% CHANGES:
% Revision 1.1  2020/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% Hst = ForwardKinematicsDH(dhparams)
%
function Hst = ForwardKinematicsDHC(dhparams)
%
    Hst = BodyDHC2tform(dhparams(1,:));  
    for i = 2:size(dhparams,1)
        Hst = Hst*BodyDHC2tform(dhparams(i,:));
    end
end
%
