%% LINK2WRENCH gets de WRENCH from the LINK AXIS and a POINT on that axis
% Use in SE(3).
%
% where the WRENCH "fo" (6x1) has the two components f (3x1) and T (3x1)
% From AXIS (3x1) & a POINT q (3X1) on that axis AT THE REFERENCE POSITION.
% It is also necessary to indicate the type of FORCETYPE ('rot' or 'tra')
% for the function to work with both ROTATION & TRANSLATION movements.
% Use in SE(3).
%
%   fo = link2wrench(ForceAxis, ForcePoint, forceType)
%
%     |f|   |       0        |
% fo =| | = |                |: for only ROTATION torque.
%     |T|   |    ForceAxis   |
%
%     |f|   |    ForceAxis    |  
% fo =| | = |                 |: for only TRANSLATION force.
%     |T|   | -ForceAxis X q  |
%
% See also: .
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
%% 	fo = link2wrench(ForceAxis, ForcePoint, forceType)
%
function fo = link2wrench(ForceAxis, ForcePoint, forceType)
%
    if forceType == "rot"
       fo = [0; 0; 0; ForceAxis];
    elseif forceType == "tra"
       fo = [ForceAxis; -cross(ForceAxis,ForcePoint)];
    else
       fo = [0; 0 ; 0: 0; 0; 0];
    end
%
    
    