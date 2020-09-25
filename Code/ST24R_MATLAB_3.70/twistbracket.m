%% "twistbracket" Computes the bracket operation to two twists.
% Use in SE(3).
%
% 	xi = twistbracket(x1,x2)
%
% Returns a twist "xi" from a the application of the bracket operation to
% the twists x1 and x2. This operation is a generalization of the cross
% product on R3 to vectors in R6.
%    |v|                              v 
% xi=| |  = [E1,E2] = [E1^*E2^-E2^*E1^] 
%    |W|         
% Con ^ wedge or twist2tform operation.
% Con v vee or tform2twist operation.
%
% See also: tform2twist, twist2tform.
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
%% x = twistbracket(x1,x2)
function xi = twistbracket(x1,x2)
%
    E1 = twist2tform(x1);
    E2 = twist2tform(x2);
    xi = tform2twist(E1*E2-E2*E1);
end
%