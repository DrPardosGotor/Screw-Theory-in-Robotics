%% "wrenchpitch" Find the pitch associate with a wrench in R^3.
% Use in SE(3).
%
% 	h = wrenchpitch(phi)
%
% Compute the pitch "h" of a wrench "xi" 6x1.
%     |f|          f'*T
% phi=| | => h = ---------
%     |T|         ||f||^2

% See also: wrenchaxis, wrenchmagnitude.
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
%% h = wrenchpitch(phi)
function h = wrenchpitch(phi)
%
    f = [phi(1,1); phi(2,1); phi(3,1)];
    T = [phi(4,1); phi(5,1); phi(6,1)];
    if norm(f) == 0
        h = inf;
    else
        h = f'*T/(norm(f)^2);
    end
end
%
 
           
