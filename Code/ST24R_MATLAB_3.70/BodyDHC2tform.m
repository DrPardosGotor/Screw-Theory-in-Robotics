%% BODYDHC2TFORM Basic Body D-H (Denavit-Hartenberg) Matrix transformation
% DHC Denavit-Hartenberg CLASSIC formulation
%
%   (Defined on mobile or tool system -> post-multiplication)
%   H = BODYDHC2TFORM(DHPARAMS) gets the homogeneous matrix H for a single
%   rigid body (link) from its DHC parameters dhparams = [d t r a] (1x4).
%       d: The translation on Z(i-1) axis (length units).
%       t: The rotation about Z(i-1) axis (radians). 
%       r: The length on Xi axis of the common normal (joint offset) in 
%          (length units).
%       a: The angle on Xi axis between successive Z(i-1) & Zi axes 
%          (joint twist) in (radians).
%   Only d and t are joint variables.
%
% See also trvZ2tform(p), rotZ2tform(g), trvX2tform(p), rotX2tform(a)
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
%% Hdh = BodyDH2tform(dhparams)
%
function Hdh = BodyDHC2tform(dhparams)
%
    d = dhparams(1);
    t = dhparams(2);
    r = dhparams(3);
    a = dhparams(4);  
    Hdh = trvZ2tform(d)*rotZ2tform(t)*trvX2tform(r)*rotX2tform(a);
end
%

