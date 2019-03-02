%% "MInertiaL" INERTIA MATRIX M(t) for an open chain manipulator.
% computation based on the use of the JstL Link Jacobian (mobil).
% Use in SE(3).
%
% 	Mt = MInertiaJsl(TwMag,LiMas)
%
% Gives the MANIPULATOR INERTIA MATRIX M corresponding to the Lagangian's
% dynamics equations: M(t)
% M(t)*ddt + C(t,dt)*dt + N(t,dt) = T
% of the robot formed by links on an open chain. 
%
% INPUTS:
% TwMag = [Tw1; Mag1, ..., Twn; Magn] (7xn)
% for each rigid body joint (1..n).
% Tw1..Twn: The TWIST components for the joint movement.
% Mag1..Magn: The MAGNITUDE component for the joint SCREW movement.
% LiMas = [CM1; IT1; Mas1, ..., CMn; ITn; Masn] (7xn)
% for each rigid body link (1..n).
% CM1..CMn: Center of Mass x-y-z Position components to S for each Link.
% IT1..ITn: Inertia x-y-z components for each Link refered to its CM.
% Mas1..Masn: The Mass for each Link.

% Ii: Diagonal Inertia Tensor (3x3 )Ixi, Iyi, and Izi are the moments of 
% inertia about the x, y, and z-axes of the ith link frame on the CM.
%
%       |M11...M1n|              i=n
% Mt =  |         |, With Mt = Sum  (JstLi'*Imi*JstLi) 
%       |Mn1...Mnn|              i=1
% 
% See also: LinkInertiaL.
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
%% MIL = MInertiaJsl(TwMag,LiMas)
%
function Mt = MInertiaJsl(TwMag,LiMas)
%
    n = size(TwMag,2);
    Mt = zeros(n);
    for i = 1:n
        Hsli0 = trvP2tform(LiMas(1:3,i));
        Ii = LiMas(4:6,i);
        mi = LiMas(7,i);        
        Im = [mi*eye(3),zeros(3);zeros(3),[Ii(1) 0 0; 0 Ii(2) 0; 0 0 Ii(3)]];
        JstL = GeoJacobianL(TwMag,Hsli0,i);
        ImL = JstL'*Im*JstL;
        Mt = Mt+ImL;
    end
end
%
        
        