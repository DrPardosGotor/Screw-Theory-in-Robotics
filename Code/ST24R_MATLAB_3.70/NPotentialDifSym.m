%% "NPotentialDifSym" Potential with the NEW GRAVITY SYMBOLIC Matrix N(t)
% for an open chain manipulator.
% computation is SYMBOLIC DIFFERENTIATION and then positirons t are valued.
% Use in SE(3).
%
% 	Nt = NPotentialSym(TwMag,LiMas,PoAcc)
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
% V(t) (3x1) potential matrix for the robot in the three axes X-Y-Z.
% Even though it is normally used only with the gravity on Z, the
% expression is general for whatever selection of axes and even for other
% types of acceleration. Imagine a mobil robot in 3D (e.g. drone) or a 
% space robot with different values and directions for gravity.
%
%     n                z
% N(ti) = d/dti(V) = - Sum  d(Mass1*PoAccj*pcm1+...+Massn*PoAccj*pcmn)/dti
%     i=1              j=x
% Where t are the generalized coordinates (joints magnitudes: t1...tn).
%
%
% See also: MInertiaJst, MInertiaAij, CCoriolisAij, NPotentialAij.
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
%% Nt = NPotentialSym(TwMag,LiMas,PoAcc)
%
function Nt = NPotentialDifSym(TwMag,LiMas,PoAcc)
%
    n = size(TwMag,2);
    syms t1 t2 t3 t4 t5 t6 t7; % maximum for a robot with 7 DOF
    ThetaSymmax = [t1 t2 t3 t4 t5 t6 t7];
    ThSym = ThetaSymmax(:,1:n);
    TwMagSym = [TwMag(1:6,1:n); ThSym];
    NtSym = ThSym';
    VtSym = zeros(3,1);
    for i = 1:n
        Hsli0 = trvP2tform(LiMas(1:3,i));
        Hslit = ForwardKinematicsPOE(TwMagSym(:,1:i))*Hsli0;
        VtSym = VtSym + diag(LiMas(7,i)*(PoAcc))*Hslit(1:3,4);
    end
    for i = 1:n
        NtSym(i) = diff(VtSym(1),ThSym(i));
        NtSym(i) = NtSym(i) + diff(VtSym(2),ThSym(i));
        NtSym(i) = NtSym(i) + diff(VtSym(3),ThSym(i));
    end
    NtSym = simplify(NtSym);
    Nt = - double(subs(NtSym, ThSym, TwMag(7,1:n)));
end
%

