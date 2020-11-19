%% "SHENGPINGTWO" Find the Inverse Kinematics of two consecutive
% DISJOINT ROTATION SCREWS which applied to a point move it in SE(3).
% Yew-Sheng & Ai-Ping TWO (YA2).
%
% 	Theta1Theta2 = ShengPingTwo(x1, x2, pp, pk)
%   by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
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
% Based on the work of Paden & Kahan subproblem two for INVERSE KINEMATICS.
% The problem could have up to TWO DOUBLE solutions for Theta1-Theta2:
% Theta1Theta2 = [t11 t21; t12 t22]
% as a consequence of the possible paths from p-c-k or p-d-k.
% The problem could have two solutions for each Theta, which is Theta-2pi, 
% but the trivial second one is not considered, even though it might be a
% valid solutions in robotics.
%              
% See also: PadenKahanTwo
% See also: PardosGotorFour
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
%% Theta1Theta2  = YewShengAiPingTwo(x1,x2,pp,pk)
%
function Theta1Theta2  = YewShengAiPingTwo(x1,x2,pp,pk)
%
    v1 = x1(1:3); w1 = x1(4:6);  
    v2 = x2(1:3); w2 = x2(4:6);
    r1 = cross(w1,v1)/(norm(w1)^2);  
    r2 = cross(w2,v2)/(norm(w2)^2);
    de = r2 - r1;
    % Calculate the a, b, g parameters for the PadenKahan2 solution
    u = pp - r2;
    v = pk - r1;
    Cw1w2 = cross(w1,w2);
    a = ((w1'*w2)*w2'*u-w1'*v)/((w1'*w2)^2-1);
    b = ((w1'*w2)*w1'*v-w2'*u)/((w1'*w2)^2-1);
    g2 = abs(real((norm(u)^2-a^2-b^2-2*a*b*w1'*w2)/norm(Cw1w2)^2));
    g = sqrt(g2);
    m2 = a*w1+b*w2+g*Cw1w2;
    n2 = a*w1+b*w2-g*Cw1w2;
    m1 = de + m2;
    n1 = de + n2;
    % Solve the TWO DOUBLE SOLUTIONS using the PadenKahanOne
    
    up2 = u-w2*w2'*u;
    mp2 = m2-w2*w2'*m2;
    mp1 = m1-w1*w1'*m1;
    np2 = n2-w2*w2'*n2;
    np1 = n1-w1*w1'*n1;
    vp1 = v-w1*w1'*v;
    if abs(norm(vp1)-norm(mp1))<0.001
        t2c = atan2(real(w2'*(cross(up2,mp2))),real(up2'*mp2));
        t1c = atan2(real(w1'*(cross(mp1,vp1))),real(mp1'*vp1));
    else
        t2c = 0; % NO SOLUTION.
        t1c = 0; % NO SOLUTION.
    end
    if abs(norm(vp1)-norm(np1))<0.001
        t2d = atan2(real(w2'*(cross(up2,np2))),real(up2'*np2));
        t1d = atan2(real(w1'*(cross(np1,vp1))),real(np1'*vp1));
    else
        t2d = 0; % NO SOLUTION.
        t1d = 0; % NO SOLUTION.
    end
    Theta1Theta2 = [t1c t2c; t1d t2d];
end 
%   
    
