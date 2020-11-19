%% "PARDOSGOTORSIX" Find the Inverse Kinematics of two consecutive
% SCREWS with SKEW axes (NOT PARALLEL)
% This is a generalization for the PK2 which needs crossing axes.
% The PG5 solves the problem for crossing and skewed axes.
% This PG5 does not work for parallel axes, which is solved with PG4.
% which applied to a point move it ot a different point in space. In SE(3).
%
% 	Theta1Theta2 = PardosGotorSix(x1, x2, pp, pk)
%
% In the case of TWO CONSECUTIVE ROTATIONS:
% Compute angles "Th2" and "Th1" of two subsequently applies SCREWS with
% corresponding twists x2 and x1, to move the point "p" to the point "k".
% Beware of the order for the movement, first applied x2 & subsequently x1.
% Point p is first moved to the point "c" or "d" by the Screw with Theta2
% and then moved from "c" or "d" to the point "k" by the Screw with Theta1.
%
% exp(E1^Theta1) * exp(E2^Theta2) * p = exp(E1^Theta1) * (c or d) = k
%          |v|
% Ei = xi =| | is 6x1 ; and p and k are points (3x1).
%          |w|
% Based on the work of Pardos subproblem for this type INVERSE KINEMATICS.
% The problem could have up to TWO DOUBLE solutions for Theta1-Theta2:
% Theta1Theta2 = [t11 t21; t12 t22]
% When crossing axes we get TWO double solutions, otherwise only one.
% as a consequence of the possible paths from p-c-k or p-d-k.
% The problem could have two solutions for each Theta, which is Theta-2pi, 
% but the trivial second one is not considered, even though it might be a
% valid solutions in robotics.
%              
% See also: PadenKahanOne, PadenKahanTwo, PadenKahanThree
% See also: PardosGotorTwo, PardosGotorThree, PardosGotorFour
% See also: PardosGotorOne
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
%% Theta1Theta2  = PardosGotorSix(x1,x2,pp,pk)
%
function Theta1Theta2  = PardosGotorSix(x1,x2,pp,pk)
%
% First we get all the characteristics of the Screws on the plane
% perpendicular to the points "pp" and "pk".
v1 = x1(1:3); w1 = x1(4:6);  
v2 = x2(1:3); w2 = x2(4:6);
r1 = cross(w1,v1)/(norm(w1)^2);  
r2 = cross(w2,v2)/(norm(w2)^2);
v = pk - r1; vw1 = w1*w1'*v; vp1 = v-vw1; nvp = norm(vp1); o1 = r1+vw1;
u = pp - r2; uw2 = w2*w2'*u; up2 = u-uw2; nup = norm(up2); o2 = r2+uw2;
%
% The core of this approach is to find the line of intersection between
% the two planes where "p" and "k" are rotating, then the intersection of
% that line with the circles described by the "p" and "k" rotations.
%
% We get Start with converting the planes to the normal form:
% we assume (as for all ST24R) that w1 and w2 are already normalized.
% the plane normal form: "w1x * X + w1y * Y + w1z * Y = d1" (same for w2)
d1 = w1'*o1;
d2 = w2'*o2;
% The intersection line of two planes is defined by the common normal
% expresed by the translation twist "x3"
% and any point on that line "r3"
v3 = cross(w1, w2);
x3 = [v3; 0; 0; 0];
w12 = w1' * w2;
r3 = (w1 * (d1 - d2 * w12) + w2 * (d2 - d1 * w12)) / (1 - w12);
% Now we take advantage of the subproblem PG3 to find the intersection
% between this line and the circles described by "k" and "p".
thv = PardosGotorThree(x3, r3, o1, nvp);
thu = PardosGotorThree(x3, r3, o2, nup);
% The intersection points will be given by adding "th(1)" & "th(2)" to "r3"
% in the direction of the line (this is the twist x3)
% beware that PG3 gives an approximation when there is no solution.
pc1 = r3 + thv(1)*v3; pd1 = r3 + thv(2)*v3;
pc2 = r3 + thu(1)*v3; pd2 = r3 + thu(2)*v3;  
%
%
% Solve the TWO DOUBLE SOLUTIONS using the PadenKahanOne approach.
m1 = pc1 - r1; mp1 = m1-w1*w1'*m1;
n1 = pd1 - r1; np1 = n1-w1*w1'*n1;
m2 = pc2 - r2; mp2 = m2-w2*w2'*m2;
n2 = pd2 - r2; np2 = n2-w2*w2'*n2;
t1c = atan2(real(w1'*(cross(mp1,vp1))),real(mp1'*vp1));
t1d = atan2(real(w1'*(cross(np1,vp1))),real(np1'*vp1));
t2c = atan2(real(w2'*(cross(up2,mp2))),real(up2'*mp2));
t2d = atan2(real(w2'*(cross(up2,np2))),real(up2'*np2));
%
    Theta1Theta2 = [t1c t2c; t1d t2d];
end 
%   
    
