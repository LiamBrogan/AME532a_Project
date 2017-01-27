function [phi,theta,psi] = DCM2Euler(DCM)
% For project, create the following functions/methods/scripts/blocks in
% C/C++/C#/Java/Perl/Python/Matlab/Simulink:

% b. DCM -> Euler Angles

phi = atan2(DCM(2,3),DCM(3,3));
theta = -asin(DCM(1,3));
psi = atan2(DCM(1,2),DCM(1,1));
 
end
