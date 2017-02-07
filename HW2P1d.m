%% HW 2 Problem 1-d) Rotational Kindematics Using quatDot = 1/2*quat*omega(Quaternion Kinematics)
%  ie. Given an orientation vector (theta vector at time=t0) and a body 
%   rate (omega vector), output a new orientation vector (theta vector at 
%   time=t0+dt.
%%
close all
clear
% clc
%% Initial Conditions
thetaVec0_BN = [pi/4, pi/6,pi/7]';
omegaVec_BE = [2 6 .7]';

t0 = 0;
dt = 0.001;
%% Quaternion Dot

[quat_BNdot,quat_BN0] = quatDot(thetaVec0_BN,omegaVec_BE);

% Set up integral to find ThetaVec1
tspan = [t0 t0+dt];
quatDot_func = @(t,y) quat_BNdot;

[t,quat_BN] = ode45(quatDot_func, tspan, quat_BN0);

[phi,theta,psi] = Quat2Euler(quat_BN(end,:)');

ThetaVec1 = [phi,theta,psi]'

function [quat_BNdot,quat0_BN] = quatDot(thetaVec,omegaVec)
%% HW 2 Problem 1-d) Rotational Kindematics Using quatDot = 1/2*quat*omega(Quaternion Kinematics)
%  ie. Given an orientation vector (theta vector at time=t0) and a body 
%   rate (omega vector), output a new orientation vector (theta vector at 
%   time=t0+dt.

phi = thetaVec(1);
theta = thetaVec(2);
psi = thetaVec(3);

quat0_BN = Euler2Quat(phi,theta,psi);
quat0_BN = quat0_BN./norm(quat0_BN);


qDot_BN0 = 0.5*dot(quat0_BN(2:4),omegaVec);
qDot_BNvec = 0.5*(quat0_BN(1)*omegaVec+cross(quat0_BN(2:4),omegaVec));

quat_BNdot = [qDot_BN0; qDot_BNvec];
end