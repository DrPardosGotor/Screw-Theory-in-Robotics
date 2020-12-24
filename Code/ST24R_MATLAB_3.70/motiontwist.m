%% "motiontwist" Find the twist which generates a rigid body motion.
% Use in SE(3).
%
% 	xi = motiontwist(H)
%
% Compute the twist xi in R^6 which generates the homogeneous matrix H 4x4.
%         |v|      |r p|
% E = xi =| | <= H=|   |
%         |w|      |0 1|
%                 |exp(W^Th)  (I-exp(W^Th))*(W x v)+W*W'*v*Theta|
% H= exp(E^Theta)=|                                             | 
%                 |0                          1                 |
%                                           
% Use Rodrigues's formula: 
% exp(W^Th)=I + W^ * sin(Th)+ W^ * W^ * (1-cos(Th)) 
% With W^=axis2skew(W)
% With W x v == cross product W by v.
% We get "v" from p as:
% v = [[(I-exp(W^Th))*W^+W*W'*Theta]^-1]*p
%
% See also: expScrew.
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
%% xi = motiontwist(H)
function xi = motiontwist(H)
%
 	r = H(1:3,1:3);
    p = H(1:3,4);
    t = rotationangle(r);
    w = rotationaxis(r,t);
    if t == 0
        t = norm(p);
        if t == 0
            v = [0;0;0];
        else
            v = p/t;  
        end
    else
        v = ((eye(3)-r)*axis2skew(w)+w*w'*t)\p;
    end 
    xi = [v;w];
end
%
