function [Roll,Pitch,Yaw] = Quat2Euler(Quat)
% For project, create the following functions/methods/scripts/blocks in
% C/C++/C#/Java/Perl/Python/Matlab/Simulink:
% c. QUAT -> Euler Angles

q0 = Quat(1);
q1 = Quat(2);
q2 = Quat(3);
q3 = Quat(4);

% atan2(2*(u(1)*u(2) + u(3)*u(4)), 1-2*(u(2)^2 + u(3)^2));
Roll = atan2(2*(q0*q1+q2*q3),1-2*(q1^2+q2^2));
if Roll > pi
    while Roll > pi
        Roll = Roll-2*pi;
    end
end
if Roll <= -pi
    while Roll <= -pi
        Roll = Roll+2*pi;
    end
end

% asin(2*(u(1)*u(3) - u(2)*u(4)));
Pitch = asin(2*(q0*q2-q3*q1));
if Pitch > pi/2
    while Pitch > pi/2
        Pitch = Pitch-pi;
    end
end
if Pitch <= -pi/2
    while Pitch <= -pi/2
        Pitch = Pitch+pi;
    end
end


% 2*(u(1)*u(4) + u(2)*u(3)), 1-2*(u(3)^2 + u(4)^2)
Yaw = atan2(2*(q0*q3+q1*q2),1-2*(q2^2+q3^2));
if Yaw > pi
    while Yaw > pi
        Yaw = Yaw-2*pi;
    end
end
if Yaw <= -pi
    while Yaw <= -pi
        Yaw = Yaw+2*pi;
    end
end

end