%% "PARDOSGOTOREIGHT" Find Inverse Kinematics of 3 consecutive ROTATIONS
% SCREWS and all of them are parallel axes.
% which applied to a pose, move it to a different pose in space SE(3).
%
% 	The123 = PardosGotorEight(x1, x2, x3, Hp, Hk)
%
% In the case of this THREE CONSECUTIVE ROTATIONS:
% Compute angles "Th3", "Th2" & "Th1" of three subsequently applied SCREWS
% with corresponding twists x3, x2 and x1, to move the pose Hp to Hk.
% the kinenatics definition of the problem is:  E1 * E2 * E3 * Hp = Hk
%
% exp(E1^The1) * exp(E2^The2) * exp(E3^The3) * Hp = Hk
%          |v|
% Ei = xi =| | is 6x1 ; and Hp and Hk are homogeneous matrix (4x4).
%          |w|
% 
% From the mechanical and robotics point of view, this canonical problems
% arises when we have 3 DoF from 3 consecutive parallel joints.
% The problem could have up to TWO TRIPLE solutions
% Theta123 = [t1co t2oc t31; t1do t2od t32]
% 
%
% Based on the Pardos-Gotor subproblems for this INVERSE KINEMATICS.
% The problem can have NONE, ONE or TWO TRIPLE solutions for Theta123:
%
% The problem could have complementary solutions for each Theta (Theta-2pi) 
% but the trivial second one is not considered, even though it might be a
% valid solutions in robotics.
%              
% See also: PadenKahanOne, PadenKahanTwo, PadenKahanThree
% See also: PardosGotorTwo, PardosGotorThree, PardosGotorOne
% See also: PardosGotorFour, PardosGotorFive, PardosGotorSix,
% See also: PardosGotorSeven
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
%% The123  = PardosGotorEight(x1, x2, x3, Hp, Hk)
%
function The123  = PardosGotorEight(x1, x2, x3, Hp, Hk)
%
% First we get all the characteristics of the Screws on the plane.
v3 = x3(1:3); w3 = x3(4:6); r3 = cross(w3,v3)/(norm(w3)^2);
pp = Hp(1:3,4); u = pp - r3; o3p = r3+w3*w3'*u;
pk = Hk(1:3,4);
%
% The core of this approach is to solve FIRST the IK for the t12 to move o3
% which has none, one or TWO double solutions.
% Kinematics problem definition: E1*E2*E3 * Hp = Hk.
% then, E1*E2*E3 * o3p = Hk * Hp^-1 * o3p = o3k
% o3p is not affected by E3,because the point is in the axis of E3. Then
% E1*E2 * o3p = o3k, which is exactly the canonical PARDOS-GOTOR-FOUR.
o3kh = Hk * (Hp \ [o3p; 1]); o3k = o3kh(1:3);
t12 = PardosGotorFour(x1, x2, o3p, o3k);
%
% Solve for the t123 SOLUTION, which is the TOTAL rotation for pp using the
% PadenKahanOne PK1 approach. Pay attention, because this t123 total
% magnitude is the addition of the three rotations (t123 = t1+t2+t3).
% As we already know the two possible double solutions for t12, the value
% for the t3 is obtained from the substaction from the total.
% 
x3k = joint2twist(w3, o3k, 'rot');
ppk = o3k + (pp - o3p);
t123 = PadenKahanOne(x3k, ppk, pk);
t31 = t123 - t12(1,1) - t12(1,2);
t32 = t123 - t12(2,1) - t12(2,2);
%
% Only need to compose the output matrix with the solutions
The123 = [t12(1,:) t31; t12(2,:) t32];
end 
%   
    
