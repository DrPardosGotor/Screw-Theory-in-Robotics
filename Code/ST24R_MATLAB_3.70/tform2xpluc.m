%% tform2xpluc Convert an homogeneous transformation (4x4) to
% a Plucker transformation (6x6)
% X Plucker tansformation = tform2spvec(H) converts an homegeneous
% transformation Hst(4x4) = [R(3x3) t(3x1); 0 1] to a
% Xst(6x6) = [E(3x3)' 0; rXE E']
% with coordinates allways in a S (spatial or original) system. 
% where "rX" is the skew symmetric matrix of the vector r
% All units in radians.
%
%   Example:
%
% See also ,
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
%% xpluc = tform2xpluc(H)
%
function X = tform2xpluc(H)
%
    X = zeros(6);
    ET = H(1:3,1:3);
    r = H(1:3,4);
    X(1:3,1:3) = ET;
    X(4:6,4:6) = ET;
    X(4:6,1:3) = axis2skew(r)*ET;
%
end

