function Quat = Euler2Quat(ThetaVec)
% For project, create the following functions/methods/scripts/blocks in
% C/C++/C#/Java/Perl/Python/Matlab/Simulink:
% b. Euler Angles (Roll, Pitch, Yaw) -> QUAT

Roll  = ThetaVec(1);
Pitch = ThetaVec(2);
Yaw   = ThetaVec(3);

Quat = [cos(Roll/2)*cos(Pitch/2)*cos(Yaw/2)+sin(Roll/2)*sin(Pitch/2)*sin(Yaw/2)
        sin(Roll/2)*cos(Pitch/2)*cos(Yaw/2)-cos(Roll/2)*sin(Pitch/2)*sin(Yaw/2)
        cos(Roll/2)*sin(Pitch/2)*cos(Yaw/2)+sin(Roll/2)*cos(Pitch/2)*sin(Yaw/2)
        cos(Roll/2)*cos(Pitch/2)*sin(Yaw/2)-sin(Roll/2)*sin(Pitch/2)*cos(Yaw/2)];
    
Quat = Quat/norm(Quat);
    
end