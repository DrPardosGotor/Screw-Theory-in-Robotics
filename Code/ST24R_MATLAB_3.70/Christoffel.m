%% "Christoffel" Christoffel Symbols for Coriolis matrix C(t,dt)
% for an open chain manipulator.
% computation based on the Adjoint Transformation Aij.
% Use in SE(3).
%
% 	dMt = Christoffel(TwMag,LiMas,I,J,K)
%
% Gives the CHRISTOFFEL SYMBOLS (t,dt) for the Lagangian's equations: 
% M(t)*ddt + C(t,dt)*dt + N(t,dt) = T
% of the dynamics of the robot formed by links on an open chain. 
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
% I, J, K: index for Cristoffel formulation.
% 
% dMt = dMij/dtk = Sum(l=max(i,j),n)[[Aki*Ei,Ek]'*Alk'*Ml*Alj*Ej + Ei'*Ali'*Ml*Alk*[Akj*Ej,Ek]]
% 
% With Ml being the link inertia nxn LinkInertiaS. 
% With Ei being the twist xi 6x1.
% With Aij being an element 6x6 of the adjoint transformation Aij2adjoint
%
% See also: LinkInertiaS, Aij2adjoint,. CCoriolisAij.
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
%% dMt = Christoffel(TwMag,LiMas,I,J,K)
%
function dMt = Christoffel(TwMag,LiMas,i,j,k)
%
    n = size(TwMag,2);    
    m = max(i,j);
    dMt = 0;
    for l = m:n
        Hsli0 = trvP2tform(LiMas(1:3,l));
        Ii = LiMas(4:6,l);
        mi = LiMas(7,l);
        dMt1 = twistbracket(Aij2adjoint(k,i,TwMag)*TwMag(1:6,i),TwMag(1:6,k))';
        dMt1 = dMt1 * Aij2adjoint(l,k,TwMag)'*LinkInertiaS(Hsli0,Ii,mi);
        dMt1 = dMt1 * Aij2adjoint(l,j,TwMag)*TwMag(1:6,j);
        dMt2 = TwMag(1:6,i)'*Aij2adjoint(l,i,TwMag)';
        dMt2 = dMt2 *LinkInertiaS(Hsli0,Ii,mi)*Aij2adjoint(l,k,TwMag);
        dMt2 = dMt2 *twistbracket(Aij2adjoint(k,j,TwMag)*TwMag(1:6,j),TwMag(1:6,k));
        dMt = dMt + dMt1 + dMt2;
    end
end
%

