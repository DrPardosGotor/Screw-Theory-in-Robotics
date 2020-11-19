%% "PARDOSGOTORFIVE" PG5 Inverse Kinematics of a single ROTATION SCREW
% when applied to a perpendicular LINE or PLANE in SE(3) for ORIENTATION.
% This PG5 is an extension of PK1 in application to lines and planes.
%
% 	Theta1 = PardosGotorFive(x1, pp, pk)
%   by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% Compute the magnitude "Theta" (angle) of the Screw, to
% move the point "p" from its original position to the point "c" or "d" by
% the ROTATION of the Twist x1.
%
%         |v|
% E = x1 =| | (6x1); and the points p and k are (3x1) coordinates. 
%         |w|
% exp(E^Theta) * p = k
%
% For a SCREW of pure ROTATION:
% Based on the work of Paden & Kahan subproblem ONE for INVERSE KINEMATICS.
% For a SCREW of pure ROTATION the problem has 2 solutions t11 and t11-2pi, 
% but the trivial second one is not considered, even though it might be a
% valid solutions in robotics.
% Nonetheless, for this case of RE-ORIENTATION of a LINE or PLANE the
% almost trivial solution is valid and many times useful, (t12=t11-pi).
%
% IDEA: In fact it computes "Theta" to rotate the vector u' around x1
% to make it parallel to the vector v', but "p" and "k" can be in
% different planes and even u' and v' be of different module, but o course
% both planes are parallel & perpendicular to the axis of x1, then and
% can not coincide after the movement, but the algorithm works.
%
% See also: PadenKahanOne, PadenKahanTwo, PadenKahanThree
% See also: PardosGotorOne, PardosGotorTwo, PardosGotorThree
% See also: PardosFotorFour, PardosGotorSix, PardosGotorSeven
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
%% Theta1 = PardosGotorFive(x1, pp, pk)
%
function Theta1 = PardosGotorFive(x1, pp, pk)
%
    v1 = x1(1:3); w1 = x1(4:6);
    r1 = cross(w1,v1)/(norm(w1)^2);
    u = pp-r1; up = u-w1*w1'*u;
    v = pk-r1; vp = v-w1*w1'*v;
    t11 = atan2(w1'*(cross(up,vp)),up'*vp);
    t12 = -sign(t11)*(pi-abs(t11));
    Theta1 = [t11 t12];
end
 %   
    
