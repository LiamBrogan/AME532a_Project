function [ thetaDotVec ] = omegaDot2thetaDot( ThetaVec, omegaVec)
%% HW 2 Problem 1-b) Rotational Kinematics Using thetaDot = "DCM" times omegaector (Euler Kinematics - the first method)
%  ie. Given an orientation vector (theta vector at time=t0) and a body 
%   rate (omega vector), output a new orientation vector (theta vector at 
%   time=t0+dt

phi = ThetaVec(1);
theta = ThetaVec(2);
psi = ThetaVec(3);

% Not really a DCM, because ("DCM")' ~= inv("DCM")
% However, this "DCM serves the same purpose for rotating angle vectors
DCM = [1 sin(phi)*tan(theta)  cos(phi)*tan(theta)
       0 cos(phi)            -sin(phi)
       0 sin(phi)/cos(theta)  cos(phi)/cos(theta)];

% thetaDotVec = [PhiDot ThetaDot PsiDot];
thetaDotVec = DCM*omegaVec;
end

