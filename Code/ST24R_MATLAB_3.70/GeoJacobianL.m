%% "GeoJacobianT" computes Geometric LINK TOOL Jacobian
% the coordinate system is refered to the Link frame.
% Use in SE(3).
%
% 	JslT = GeoJacobianL(TwMag,Hsl0,Li)
%
% GEOMETRIC JACOBIAN in a LINK frame: At each configuration of theta,
% maps the joint velocity vector, into the corresponding velocity of some
% robot LINK.
% The "ith" column of jst is the "ith" joint twist, written with respect
% to the LINK frame at to the current manipulator configuration.
%
% INPUTS:
% TwMag = [Tw1; Mag1, ..., Twn; Magn] (7xn)
% for each rigid body joint (link 1..n).
% Tw1..Twn: The TWIST components for the joint SCREW movement.
% Mag1..Magn: The MAGNITUDE component for the joint SCREW movement.
% Hsl0 is the pose of the link frame "L", associated to the Center o Mass
% of that link at the reference (home) configuration or the manipulator.
% Li: link number.
%
% JslL = (Ad(Hsl0)^-1)*[Ai1*E1 ... Aii*Ei 0 ... 0] (6xn)
% where Aij = Aij2Adjoint and En are the Twists.
%
% See also: GeoJacobianS, GeoJacobianT, Aij2adjoint, tform2adjoint.
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
%% JslT = GeoJacobianL(TwMag,Hsli0,Li)
%
function JslT = GeoJacobianL(TwMag,Hsli0,Li)
%
    n = size(TwMag,2);
    JslT = zeros(6,n);
    AdHsli0 = tform2adjoint(Hsli0);
    for j = 1:n
        JslT(:,j) = AdHsli0\(Aij2adjoint(Li,j,TwMag)*TwMag(1:6,j));
    end
end
%