%% BODYDHM2TFORM Basic Body D-H (Denavit-Hartenberg) Matrix transformation
% DHC Denavit-Hartenberg MODIFIED formulation
%
%   (Defined on mobile or tool system -> post-multiplication)
%   H = BODYDHM2TFORM(DHMPARAMS) gets the homogeneous matrix H for a single
%   rigid body (link) from its DHM parameters dhparams = [a r t d] (1x4).  
%       a: The angle on Xi-1 axis between successive Z(i-1) & Zi axes 
%          (joint twist) in (radians).
%       r: The length on Xi-1 axis of the common normal (joint offset) in 
%          (length units).
%       t: The rotation about Zi axis (radians).
%       d: The translation on Zi axis (length units).
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
%% Hdh = BodyDHM2tform(dhmparams)
%
function Hdh = BodyDHM2tform(dhmparams)
%
    a = dhmparams(1);
    r = dhmparams(2);
    t = dhmparams(3);
    d = dhmparams(4);
    Hdh = rotX2tform(a)*trvX2tform(r)*rotZ2tform(t)*trvZ2tform(d);
end
%

