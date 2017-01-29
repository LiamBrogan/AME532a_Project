%% HW 2 Problem 1-b) Rotational Kinematics Using thetaDot = "DCM" times omegaector (Euler Kinematics - the first method)
%  ie. Given an orientation vector (theta vector at time=t0) and a body 
%   rate (omega vector), output a new orientation vector (theta vector at 
%   time=t0+dt
%%
close all
clear
clc
%%
thetaVec0 = [pi/4, pi/6,pi/7]';
omegaVec = [2 6 .7]';

t0 = 0;
dt = 0.001;

ThetaDotVec = omegaDot2thetaDot( thetaVec0, omegaVec);

% Set up integral to find ThetaVec1
tspan = [t0 t0+dt];
ThetaDot = @(t,y) ThetaDotVec;

[t,ThetaVec] = ode45(ThetaDot, tspan, thetaVec0);

ThetaVec1 = ThetaVec(end,:)'