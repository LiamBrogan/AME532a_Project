%% HW 2 Problem 1-d) Rotational Kindematics Using quatDot = 1/2*quat*omega(Quaternion Kinematics)
%  ie. Given an orientation vector (theta vector at time=t0) and a body 
%   rate (omega vector), output a new orientation vector (theta vector at 
%   time=t0+dt.
%%
close all
clear
% clc
%% Initial Conditions
thetaVec0 = [pi/4, pi/6,pi/7]';
omegaVec = [2 6 .7]';

t0 = 0;
dt = 0.001;
%% Quaternion Dot

[quat_BNdot,quat_BN0] = quatDot(thetaVec0,omegaVec);

% Set up integral to find ThetaVec1
tspan = [t0 t0+dt];
quatDot_func = @(t,y) quat_BNdot;

[t,quat_BN] = ode45(quatDot_func, tspan, quat_BN0);

[phi,theta,psi] = Quat2Euler(quat_BN(end,:)');

ThetaVec1 = [phi,theta,psi]'