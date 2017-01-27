function V2 = QuatRot(V1,Quat)
% For project, create the following functions/methods/scripts/blocks in
% C/C++/C#/Java/Perl/Python/Matlab/Simulink:
% d. Coordinate transforms using QUAT
% ie, given vector “V” in 1 and QUAT to 2 from 1 (q2/1), provide vector “V” in 2

invQuat = 1/norm(Quat)*[Quat(1); -Quat(2:4)];
V1quat = [0; V1];

V2quat = QuatMult(QuatMult(invQuat,V1quat),Quat);
V2 = V2quat(2:4);

end