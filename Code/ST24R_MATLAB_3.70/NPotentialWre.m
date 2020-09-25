%% "NPotentialWre" Potential with the NEW GRAVITY WRENCH Matrix N(t)
% for an open chain manipulator.
% computation based on the use of the gravity WRENCH on the robot links.
% Use in SE(3).
%
% A N(t) new formulation in ST24R by Dr. Pardos-Gotor
% It allows to get the Potential matrix in Dynamics, avoiding
% the differentiation of the Potential Energy V(t).
%
% 	Nt = NPotentialWre(TwMag,LiMas,PoAcc)
%
% Gives the POTENTIAL MATRIX N(t) for the Lagangian's equations: 
% M(t)*ddt + C(t,dt)*dt + N(t) = T
% of the dynamics of the robot formed by links on an open chain.
% It does not consider friction N(t, dt) and only gravitational N(t).
%
% INPUTS:
% TwMag = [Tw1; Mag1, ..., Twn; Magn] (7xn)
% for each rigid body joint (1..n).
% Tw1..Twn: The TWIST components for the joint movement.
% Mag1..Magn: The MAGNITUDE component for the joint SCREW movement.
% LiMas = [CM1; IT1; Mass1, ..., CMn; ITn; Massn] (7xn)
% for each rigid body link (1..n).
% CM1..CMn: Center of Mass x-y-z Position components to S for each Link.
% IT1..ITn: Inertia x-y-z components for each Link refered to its CM.
% Mass1..Massn: The Mass for each Link.
% PoAcc = Vector (3x1) with the accelerations for potential energies.
% e.g PoAcc = [0 0 -9.81] if the only acceleration is gravity on -Z axis.
%
% OUTPUTS:
% N(t) (nx1) potential matrix for the dynamics expression.
% Even though it is normally used only with the gravity on Z, the
% expression is genral for whatever selection of axes and even for other
% types of acceleration. Imagine a mobil robot in 3D (e.g. drone) or a 
% space robot with different values and directions for gravity.
%
%      |Nt1|          n  
% Nt = |   |; Nti = - Sum  JstSi*Fgi
%      |Ntn|          1
%
% with JslSi the Spatial Jacobian affecting the link i.
%
% with Fgi the GRAVITY WRENCH (pure translational) in the SPATIAL frame.
%           |  w |      |   wg  |  |        wg          |    
% Fgi = mi*g|    |= mi*g|       |= |                    |       
%           |-wxq|      |-wgxCMi|  |-wg x POE(1:i)*Hsli0|    
% wg is the gravity axis application.
% CMi center of mass for the link i, calculated for the current pose.
% 
% See also: MInertiaAij, CCoriolisAij, NPotentialAij.
% See also: MInertiaJsl, NPotentialDifSym.
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
%% Nt = NPotentialWre(TwMag,LiMas,PoAcc)
%
function Nt = NPotentialWre(TwMag,LiMas,PoAcc)
%
    n = size(TwMag,2);    
    forcetype = 'tra';
    MagnG = norm(PoAcc);
    AxisG = PoAcc/MagnG;
    Wrench = zeros(6,n);
    for i = 1:n
        Hsli0 = trvP2tform(LiMas(1:3,i));
        Hslit = ForwardKinematicsPOE(TwMag(:,1:i))*Hsli0;
        mi = LiMas(7,i);
        Wrench(:,i) = mi*MagnG*(link2wrench(AxisG, Hslit(1:3,4), forcetype));
    end
    Nt = zeros(n,1);
    for i = 1:n  
        JstS = zeros(6,n);
        JstS(:,1:i) = GeoJacobianS(TwMag(:,1:i));
        Ntnew = JstS'*Wrench(:,i);
        Nt = Nt - Ntnew;
    end
end
%

