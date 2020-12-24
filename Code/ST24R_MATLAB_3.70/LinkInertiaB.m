%% "linkInertiaB" the LINK INERTIA MATRIX in the Body coordinate frame.
% When working with SPATIAL VECTOR ALGEBRA.
%
% 	ImB = LinkInertiaB(CMi,Ii,mi)
%
% It gives the LINK INERTIA MATRIX corresponding to the inertia
% of the link into the same BODY frame of the link.
%
% INPUTS:
% CMi: Link Center of Mass meassured in the same Link coordinate frame.
% Ii: Inertia Tensor (3x3) = formed bye the moments of inertia (diagonal)
% and the products of inertia.
% inertia about the x, y, and z-axes of the ith link frame on the CM.
% mi: link mass.
%
% See also: .
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
%% ImB = LinkInertiaB(CMi,Ii,mi)
%
function ImB = LinkInertiaB(CMi,Ii,mi)
%
    CM = axis2skew(CMi);
    ImB = [Ii + mi*(CM*CM'), mi*CM; mi*CM', mi*eye(3)];
end
%   
