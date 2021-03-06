function C_BN = DCM_NED2Body(ThetaVec)
% For project, create the following functions/methods/scripts/blocks in
% C/C++/C#/Java/Perl/Python/Matlab/Simulink:

% a. Euler Angles (Roll, Pitch, Yaw) -> DCM

phi  = ThetaVec(1);
theta = ThetaVec(2);
psi   = ThetaVec(3);

C_BN = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta)
       -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi) cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi) sin(phi)*cos(theta)
        sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi) -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi) cos(phi)*cos(theta)];
end