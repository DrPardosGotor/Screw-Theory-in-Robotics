%% "linkInertiaS" the LINK TRANSFORMED INERTIA MATRIX for robot's link.
% Use in SE(3).
%
% 	ImS = LinkInertiaS(Hsli0,Ii,mi)
%
% It gives the LINK TRANSFORMED INERTIA MATRIX corresponding to the inertia
% of the link into the base frame of the manipulator.
%
% INPUTS:
% Hsli0 is the homogeneous matrix 4x4 for the pose of the i link frame "L", 
% associated to the Center o Mass, at the reference (home) configuration 
% of the manipulator.
% Ii: Inertia Tensor (3x1) = [Ixi; Iyi; Izi] are the moments of 
% inertia about the x, y, and z-axes of the ith link frame on the CM.
% mi: link mass.
%
% the Inertia matrix "ImS" is nxn with n the number of links (joints).
% (Ad(Hsl0)^-1)*         
% AdHsl0 = Ad(Hsl0^-1)= Adjoint transformation of the inverse of the 
% center of mass ref config.
%                      |mI  0|
% ImS = (Ad(Hsl0)^-1)'*|     |*(Ad(Hsl0)^-1)=(Ad(Hsl0)^-1)'*M*(Ad(Hsl0)^-1)
%                      |0  Ii|
% ImS = Link Transformed Inertia Matrix.
% with M Generalized Inertia Matrix, defined through the diagonal mass and
% the inertia tensor.
%
% See also: tform2adjoint.
%
% Copyright (C) 2001-2019, by Dr. Jose M. Pardos-Gotor.
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
% Revision 1.1  2019/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% ImS = LinkInertiaS(Hsli0,Ii,mi)
%
function ImS = LinkInertiaS(Hsli0,Ii,mi)
%
    AI = eye(6);
    Im = [mi*eye(3),zeros(3);zeros(3),[Ii(1) 0 0; 0 Ii(2) 0; 0 0 Ii(3)]];
    AdHsli0 = tform2adjoint(Hsli0)\AI; % Ad(Hsli0)^-1
    ImS = AdHsli0'*Im*AdHsli0;
end
%   
