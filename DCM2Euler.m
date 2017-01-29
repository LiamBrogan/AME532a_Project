function [phi,theta,psi] = DCM2Euler(C_BN)
% For project, create the following functions/methods/scripts/blocks in
% C/C++/C#/Java/Perl/Python/Matlab/Simulink:

% b. DCM -> Euler Angles

phi = atan2(C_BN(2,3),C_BN(3,3));
theta = -asin(C_BN(1,3));
psi = atan2(C_BN(1,2),C_BN(1,1));
 
end
