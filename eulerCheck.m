function [ theta2 ] = eulerCheck( theta )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

Roll = theta(1);
Pitch = theta(2);
Yaw = theta(3);

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

theta2 = [Roll,Pitch,Yaw];


end

