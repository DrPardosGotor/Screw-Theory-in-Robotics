%% JOINTMAG2LIMITS gets the MAGNITUDE for the joint inside mechanical LIMITS
% Use in SE(3).
%
% ThTheory = Ideal joint magnitude.
% LimitPos = maximun positive magnitude.
% LimitNeg = maximum negative magnitude.
% Use in SE(3).
%
%   Theta = jointmag2limits(TheTheory, LimitPos, LimitNeg)
%
% See also: .
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
%% 	Theta = jointmag2limits(TheTheory, LimitPos, LimitNeg)
%
function Theta = jointmag2limits(TheTheory, LimitPos, LimitNeg)
%
    JointVal = sort([LimitPos; TheTheory; LimitNeg]);
    Theta = JointVal(2,:);
end
%
    