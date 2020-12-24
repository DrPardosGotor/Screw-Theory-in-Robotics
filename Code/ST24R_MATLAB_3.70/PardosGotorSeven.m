%% "PARDOSGOTORSEVEN" Find Inverse Kinematics of 3 consecutive ROTATIONS
% SCREWS with one SKEW axis (NOT PARALLEL) and then two parallel axes.
% which applied to a point move it ot a different point in space. In SE(3).
%
% 	The123 = PardosGotorSeven(x1, x2, x3, pp, pk)
%
% In the case of this THREE CONSECUTIVE ROTATIONS:
% Compute angles "Th3", "Th2" and "Th1" of two subsequently applies SCREWS
% with corresponding twists x3, x2 and x1, to move the point "p" to "k".
% Beware of the order for the movement: first X3 + x2 & subsequently x1.
% Point p is first moved to the point "e" or "f" by the Screw with Theta3
% then moved to the point "c" or "d" by the Screw with Theta2
% and then moved from "c" or "d" to the point "k" by the Screw with Theta1.
%
% exp(E1^The1) * exp(E2^The2) * exp(E3^The3) * p = k
%          |v|
% Ei = xi =| | is 6x1 ; and p and k are points (3x1).
%          |w|
% 
% Based on the Pardos-Gotor subproblems for this INVERSE KINEMATICS.
% The problem could have up to FOUR TRIPLE solutions for Theta123:
% Theta123 = [t1ck t2ec t3pe; t1ck t2fc t3pf; t1dk t2ed t3pe; t1dk t2fd t3pf;]
% as a consequence of paths: p-e-c-k, p-f-c-k, p-e-d-k, p-f-d-k.
% The problem could have complementary solutions for each Theta (Theta-2pi) 
% but the trivial second one is not considered, even though it might be a
% valid solutions in robotics.
%              
% See also: PadenKahanOne, PadenKahanTwo, PadenKahanThree
% See also: PardosGotorTwo, PardosGotorThree, PardosGotorOne
% See also: PardosGotorFour, PardosGotorFive
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
%% The123  = PardosGotorSeven(x1, x2, x3, pp, pk)
%
function The123  = PardosGotorSeven(x1, x2, x3, pp, pk)
%
% First we get all the characteristics of the Screws on the plane
% perpendicular to the points "pp" and "pk".
v1 = x1(1:3); w1 = x1(4:6);  
v3 = x3(1:3); w3 = x3(4:6);
r1 = cross(w1,v1)/(norm(w1)^2);  
r3 = cross(w3,v3)/(norm(w3)^2);
v = pk - r1; vw1 = w1*w1'*v; vp1 = v-vw1; nvp = norm(vp1); o1 = r1+vw1;
u = pp - r3; uw3 = w3*w3'*u; up3 = u-uw3; nup = norm(up3); o3 = r3+uw3;
%
% The core of this approach is to find the line of intersection between
% the two planes where "p" and "k" are rotating, then the intersection of
% that line with the circle described by the "k" rotation.
%
% We get Start with converting the planes to the normal form:
% we assume (as for all ST24R) that w1 and w3 are already normalized.
% the plane normal form: "w1x * X + w1y * Y + w1z * Y = d1" (same for w3)
d1 = w1'*o1;
d3 = w3'*o3;
% The intersection line of two planes is defined by the common normal
% expresed by the translation twist "x4"
% and any point on that line "r4"
v4 = cross(w1, w3);
x4 = [v4; 0; 0; 0];
w13 = w1' * w3;
r4 = (w1 * (d1 - d3 * w13) + w3 * (d3 - d1 * w13)) / (1 - w13);
%
% Now we take advantage of the subproblem PG3 to find the intersection
% between this line and the circle described by "k".
t4 = PardosGotorThree(x4, r4, o1, nvp);
% The intersection points will be given by adding "t4(1)" & "t4(2)" to "r4"
% in the direction of the line (this is the twist x4)
% beware that PG3 gives an approximation when there is no solution.
pc = r4 + t4(1)*v4; pd = r4 + t4(2)*v4;
%
%
% Solve the TWO SOLUTIONS using the PadenKahanOne approach.
m1 = pc - r1; mp1 = m1-w1*w1'*m1;
n1 = pd - r1; np1 = n1-w1*w1'*n1;
t1ck = atan2(real(w1'*(cross(mp1,vp1))),real(mp1'*vp1));
t1dk = atan2(real(w1'*(cross(np1,vp1))),real(np1'*vp1));
%
% Solve for TWO DOUBLE SOLUTIONS using the PardosGotorFour approach for
% each of the t1 values.
t23c = PardosGotorFour(x2, x3, pp, pc);
t23d = PardosGotorFour(x2, x3, pp, pd);
%
% Only need to compose the output matrix with the solutions
The123 = [t1ck t23c(1,:); t1ck t23c(2,:); t1dk t23d(1,:); t1dk t23d(2,:);];
end 
%   
    
