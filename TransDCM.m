function Vec_b = TransDCM(Vec_a,DCM_ba)

% For project, create the following functions/methods/scripts/blocks in
% C/C++/C#/Java/Perl/Python/Matlab/Simulink:

% c. Coordinate transforms using DCM
%        i. ie, given vector “V” in 1 and DCM to 2 from 1 (C2/1), provide vector “V” in 2

Vec_b = DCM_ba*Vec_a;
end