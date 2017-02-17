function V2 = QuatRot(V1,Quat)
% For project, create the following functions/methods/scripts/blocks in
% C/C++/C#/Java/Perl/Python/Matlab/Simulink:
% d. Coordinate transforms using QUAT
% ie, given vector “V” in 1 and QUAT to 2 from 1 (q2/1), provide vector “V” in 2

qo = Quat(1);
qvec = Quat(2:4);

V2 = 2*qvec*(dot(qvec,V1))+(qo^2-dot(qvec,qvec))*V1 +2*qo*cross(V1,qvec);

end