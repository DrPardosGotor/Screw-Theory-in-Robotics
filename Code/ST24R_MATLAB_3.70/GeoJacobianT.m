%% "GeoJacobianT" computes Geometric Jacobian refered to the Tool frame
% Use in SE(3).
%
% 	JstT = GeoJacobianT(TwMag,Hst0)
%
% GEOMETRIC JACOBIAN in TOOL frame: At each configuration of theta,
% maps the joint velocity vector, into the corresponding velocity of the
% end effector TOOL.
% The "ith" column of jst is the "ith" joint twist, written with respect
% to the tool frame at to the currentmanipulator configuration.
%
% INPUTS:
% TwMag = [Tw1; Mag1, ..., Twn; Magn] (7xn)
% for each rigid body joint (link 1..n).
% Twn1..Twn6: The TWIST components for the joint SCREW movement.
% Magn: The MAGNITUDE component for the joint SCREW movement.
% Hst0 is th location of the tool frame "T" at the reference configuration.
%
%               |v1'' v2'' ... vn''| 
% JstT(theta) = |                  |   
%               |w1'' w2'' ... wn''|  
%            |vi''|  
% With: Ei''=|    |=Ad^-1                                      *Ei
%            |wi''|    (exp(Ei^thetai)*...*exp(En^thetan)*Hst0)
%
%
% See also: GeoJacobianS, GeoJacobianL, expScrew, tform2adjoint.
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
%% JstT = GeoJacobianT(TwMag,Hst0)
%
function JstT = GeoJacobianT(TwMag,Hst0)
%
    JstT = TwMag(1:6,:);
    PoE = Hst0;
    for i = size(TwMag,2):-1:1
        PoE = expScrew(TwMag(:,i))*PoE;
        JstT(:,i) = tform2adjoint(PoE)\TwMag(1:6,i);
    end
end
%