%% TRVP2TFORM Convert to Homogeneous matrix a translation P vector 
%  Hp = TRVP2TFORM(P) converts a translation P axis into the
%  corresponding homogeneous matrix H. P is a position in longitude units.
%
%   Example:
%       %Calculate the homogeneous matrix for a translation p = [px;py;pz]
%       on X axis.
%       Hxp = trvP2tform(p)
%       % Hp = [1 0 0 px; 0 1 0 py; 0 0 1 pz; 0 0 0 1] 
%       ans =
%                1         0         0         px
%                0         1         0         py
%                0         0         1         pz
%                0         0         0         1
%
% See also trvY2tform(p), trvY2tform(p), trvZ2tform(p).
%
% Copyright (C) 2003-2019, by Dr. Jose M. Pardos-Gotor.
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
%% Hxp = trvX2tform(pg)
%
function Hp = trvP2tform(p)
%
    Hp = [1 0 0 p(1,1); 0 1 0 p(2,1); 0 0 1 p(3,1); 0 0 0 1];
end
%

