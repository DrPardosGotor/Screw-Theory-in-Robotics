%% "twistmagnitude" Find the magnitude of a twist in R^3.
% Use in SE(3).
%
% 	m = twistmagnitude(xi)
%
% Compute the magnitude "M" of a twist "xi" 6x1.
%     |v|          
% xi =| | => M = ||w||, if w is not ZERO.
%     |w|        
%     |v|          
% xi =| | => M = ||v||, if w = 0.
%     |w|        
% Be careful, because this definition is an extension for defining a SCREW 
% associated with a twist, but It does not mean that the twist is a screw
% with the translation component parallel to the rotation component.
%
% See also: twistpitch, twistaxis.
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
%%  m = twistmagnitude(xi)
function m = twistmagnitude(xi)
%
    v = [xi(1,1); xi(2,1); xi(3,1)];
    w = [xi(4,1); xi(5,1); xi(6,1)];
    if norm(w) == 0
        m = norm(v);
    else
        m = norm(w);
    end
end
%
    
