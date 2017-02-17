function [quat_BNdot] = quatDot(thetaVec,omegaVec)
%% HW 2 Problem 1-d) Rotational Kindematics Using quatDot = 1/2*quat*omega(Quaternion Kinematics)
%  ie. Given an orientation vector (theta vector at time=t0) and a body 
%   rate (omega vector), output a new orientation vector (theta vector at 
%   time=t0+dt.

quat0_BN = Euler2Quat(thetaVec);
quat0_BN = quat0_BN./norm(quat0_BN);


qDot_BN0 = 0.5*dot(quat0_BN(2:4),omegaVec);
qDot_BNvec = 0.5*(quat0_BN(1)*omegaVec+cross(quat0_BN(2:4),omegaVec));

quat_BNdot = [qDot_BN0; qDot_BNvec];
% quat_BNdot = quat_BNdot./norm(quat_BNdot);


end

