%% "wrenchmagnitude" Find the magnitude of a wrench in R^3.
% Use in SE(3).
%
% 	m = wrenchmagnitude(phi)
%
% Compute the magnitude "M" of a wrench "phi" 6x1.
%     |f|          
% xi =| | => M = ||f||, if f is not ZERO.
%     |T|        
%     |f|          
% xi =| | => M = ||T||, if f = 0.
%     |T|        
%
% See also: wrenchaxis, wrenchpitch.
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
%% 
function m = wrenchmagnitude(xphi)
%
    f = [xphi(1,1); xphi(2,1); xphi(3,1)];
    T = [xphi(4,1); xphi(5,1); xphi(6,1)];
    if norm(f) == 0
        m = norm(T);
    else
        m = norm(f);
    end
end
%
    
