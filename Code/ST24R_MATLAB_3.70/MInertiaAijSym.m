%% "MInertiaAij" INERTIA MATRIX M(t) for an open chain manipulator.
% computation SYMBOLIC based on the Adjoint Transformation Aij,
% and the transformed Inertia Matrix LinkInertiaS.
%
% Use in SE(3).
%
% 	Mt = MInertiaS(TwMag,LiMas)
%
% Gives the INERTIA MATRIX M corresponding to the Lagangian's
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
%
%      |Mt11...Mt1n|            n  
% Mt = |           |; Mtij = Sum  Ei'*Ali'*LinkInertiaS*Alj*Ej
%      |Mtn1...Mtnn|            l=max(i,j)
% 
% See also: LinkInertiaS, NPotentialAij, CCoriolisAij.
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
%% Mt = MInertiaAijSym(TwMag,LiMas)
%
function Mt = MInertiaAijSym(TwMag,LiMas)
%
    n = size(TwMag,2);
    syms t1 t2 t3 t4 t5 t6 t7; % maximum for a robot with 7 DOF
    syms zero;
    ThetaSymmax = [t1 t2 t3 t4 t5 t6 t7];
    ThSym = ThetaSymmax(:,1:n);
    TwMagSym = [TwMag(1:6,1:n); ThSym];
    Mt = zeros(n);
    MtSym = Mt + zero;
    for i = 1:n
        for j = 1:n
            k = max(i,j);
            for l = k:n
                Hsli0 = trvP2tform(LiMas(1:3,l));
                Ii = LiMas(4:6,l);
                mi = LiMas(7,l);
                MISij = TwMagSym(1:6,i)'*Aij2adjoint(l,i,TwMagSym)';
                MISij = MISij*LinkInertiaS(Hsli0,Ii,mi);
                MISij = MISij*Aij2adjoint(l,j,TwMagSym)*TwMagSym(1:6,j);
                MtSym(i,j) =  MtSym(i,j) + MISij;
            end
        end
    end
    MtSym = simplify(MtSym);
    Mt = double(subs(MtSym, [zero ThSym], [0 TwMag(7,1:n)]));
end
%
        
        