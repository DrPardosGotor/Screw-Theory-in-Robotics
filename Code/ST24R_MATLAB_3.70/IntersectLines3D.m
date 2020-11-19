%% "INTERSECTLINES3D" Find intersection point of two axes in SE(3).
%
% 	m = IntersectLines3D(Axes,Points)
%   by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% Compute a point "m" (3x1) in intersectin of two lines in 3D.
% Axis is the matrix (3x2) with the Axis x1 & x2 of the two lines.
% Point is the matrix (3x2) with the Points p1 & p2 on the two lines.
%
% c = p1 ; a point on the Axis of line1.
% e = x1 ; Axis or DIRECTION VECTOR (normalized) for line1.
% d = p2 ; a point on the Axis of line2.
% f = x2 ; Axis or DIRECTION VECTOR (normalized) for line2.
% g = d - c.
%         || f x g || 
% m = c + ----------- e ; for   (f x g)'(f x e)>=0 in same direction
%         || f x e || 
%
%         || f x g || 
% m = c - ----------- e ; for  (f x g)'(f x e)<0 in different direction
%         || f x e || 
%
% See also: 
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
% Revision 1.1  2020/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% m = IntersectLines3D(Axes,Points)
%
function m = IntersectLines3D(Axes,Points)
%
    c = Points(:,1); e = Axes(:,1);
    d = Points(:,2); f = Axes(:,2);
    g = d - c;
    Cfg = cross(f,g);
    Cfe = cross(f,e);
    if Cfe == 0
        m = [inf; inf; inf];
    elseif (Cfg'*Cfe) >= 0
        m = c + (norm(Cfg)/norm(Cfe))*e;
    else 
        m = c - (norm(Cfg)/norm(Cfe))*e;
    end
 end
%   
    
