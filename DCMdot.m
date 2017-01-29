function [C_NBdot,C_NB0] = DCMdot(thetaVec,omegaVec)
%% HW 2 Problem 1-c) Rotational Kindematics Using DCMdot = Omega-cross-DCM(Poisson Kinematics or Strap-Down Equations)
%  ie. Given an orientation vector (theta vector at time=t0) and a body 
%   rate (omega vector), output a new orientation vector (theta vector at 
%   time=t0+dt.

% Euler Angles
phi = thetaVec(1);
theta = thetaVec(2);
psi = thetaVec(3);

% Body Rates
P = omegaVec(1);
Q = omegaVec(2);
R = omegaVec(3);

omegaMat = [0 -P  R
            P  0 -Q
           -R  Q  0];
       
C_BN = DCM_NED2Body(phi,theta,psi);
C_NB0 = C_BN';

C_NBdot = omegaMat*C_NB0;

end

