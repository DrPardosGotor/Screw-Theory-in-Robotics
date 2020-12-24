%% "AIJ2ADJOINT" Computes ADJOINT TRANSFORMATION for a list of twists-mag.
% Use in SE(3).
% Notation useful for Link Jacobian (mobile).
% Notation useful for Christofell Symbols.
% Use in SE(3).
%
% 	Ad = Aij2adjoint(i,j,TwMag)
%
% INPUTS:
% TwMag = [Tw1; Mag1, ..., Twn; Magn] (7xn)
% for each rigid body joint (link 1..n).
% Twn1..Twn6: The TWIST components for the joint SCREW movement.
% Magn: The MAGNITUDE component for the joint SCREW movement.
%
% ADJOINT TRANSFORMATION: This is a special notation which gives us a most 
% form of the Adjoint of an open chain manipulator
% We use this notation for an easy calculation of the Manipulator Inertia 
% Matrix and the Manipulator Coriolis Matrix. 
% Computes the Adg in R^6 (6x6 matrix) from any robot link.
%      I                                    if i=j 
% Aij= Ad^-1[(exp(Ej+1,Tj+1)...(exp(Ei,Ti)] if i>j
%      0                                    if i<j
%
% See also: tform2adjoint, expScrew.
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
%% 	Ad = Aij2adjoint(i,j,TwMag)
%
function Ad = Aij2adjoint(i,j,TwMag)
%
    AZ = zeros(6);
    AI = eye(6);
    if i<j
        Ad = AZ;
    elseif i==j
        Ad = AI;    
    else
        PoE = expScrew(TwMag(:,j+1));
        for k = j+2:i
            PoE = PoE*expScrew(TwMag(:,k)); 
        end 
        Ad = tform2adjoint(PoE)\AI;
    end
end
%
%