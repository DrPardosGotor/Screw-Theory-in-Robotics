%% JOINTMAG2LIMITS gets the MAGNITUDE for the joint inside mechanical LIMITS
% Use in SE(3).
%
% ThTheory = Ideal joint magnitude.
% LimitPos = maximun positive magnitude.
% LimitNeg = maximum negative magnitude.
% It is also necessary to indicate the type of JOINTTYPE ('rot' or 'tra')
% for the function to work with both ROTATION & TRANSLATION movements.
% Use in SE(3).
%
%   xi = Th= jointmag2limits(ThTheory, LimitPos, LimitNeg, JointType)
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
%% 	Th= jointmag2limits(ThTheory, LimitPos, LimitNeg, JointType)
%
function Th = jointmag2limits(ThTheory, LimitPos, LimitNeg, JointType)
%
    if (JointType == "rot") && (abs(ThTheory)>pi)
        ThTheory = ThTheory - sign(ThTheory)*2*pi;
    end
    if ThTheory >= 0
        if LimitPos >= ThTheory
            Th = ThTheory;
        elseif LimitNeg <= ThTheory-2*pi
            Th = ThTheory-2*pi;
        elseif (ThTheory - LimitPos) <= (LimitNeg - (ThTheory-2*pi))
            Th = LimitPos;
        else
            Th = LimitNeg;
        end
    else
        if LimitNeg <= ThTheory
            Th = ThTheory;
        elseif LimitPos >= ThTheory+2*pi
            Th = ThTheory+2*pi;
        elseif (LimitNeg - ThTheory) <= ((ThTheory+2*pi) - LimitPos)
            Th = LimitNeg;
        else
            Th = LimitPos;
        end       
    end
end
%
    
    