%% "NPotentialAij" POTENTIAL matrix N(t) for an open chain manipulator.
% computation based on the Adjoint Transformation Aij.
% Use in SE(3).
% A N(t) NEW FORMULATION completely algebraic in ST24R by Dr. Pardos-Gotor
% not seen before in the literature. It allows to get the Potential matrix
% in Dynamics, avoiding the differentiation of the Potential Energy V(t).
%
% 	Nt = NPotentialAij(TwMag,LiMas,PoAcc)
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
% e.g PoAcc = [0 0 -9.8] if the only acceleration is gravity on -Z axis.
%
% OUTPUTS:
% N(t) (nx1) potential matrix for the dynamics expression.
% Even though it is normally used only with the gravity on Z, the
% expression is genral for whatever selection of axes and even for other
% types of acceleration. Imagine a mobil robot in 3D (e.g. drone) or a 
% space robot with different values and directions for gravity.
%
%      |Nt1|          n  
% Nt = |   |; Nti = - Sum  Ei'*Ali'*LinkInertiaS*Al1*Eg
%      |Ntn|          l=i
%
% See also: MInertiaAij, CCoriolisAij, NPotentialSym, LinkInertiaS.
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
%% NPL = NPotentialAij(TwMag,LiMas,PoAcc)
%
function Nt = NPotentialAij(TwMag,LiMas,PoAcc)
%
    n = size(TwMag,2);
    Eg = [PoAcc; 0; 0; 0];
    Nt = zeros(n,1);
    for i = 1:n
        for l = i:n
            Hsli0 = trvP2tform(LiMas(1:3,l));
            Ii = LiMas(4:6,l);
            mi = LiMas(7,l);
            Nti = TwMag(1:6,i)'*Aij2adjoint(l,i,TwMag)';
            Nti = Nti*LinkInertiaS(Hsli0,Ii,mi);
            Nti = Nti*Aij2adjoint(l,1,TwMag)*Eg;
            Nt(i) = Nt(i) - Nti;
        end
    end
end
%

