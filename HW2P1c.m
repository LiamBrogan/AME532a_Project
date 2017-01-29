%% HW 2 Problem 1-c) Rotational Kindematics Using DCMdot = Omega-cross-DCM(Poisson Kinematics or Strap-Down Equations)
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
%% Strapdown Equations

[C_NBdot,C_NB0] = DCMdot(thetaVec0,omegaVec);

% Set up integral to find ThetaVec1
tspan = [t0 t0+dt];
DCMdot_func1 = @(t,y) C_NBdot(:,1);
DCMdot_func2 = @(t,y) C_NBdot(:,2);
DCMdot_func3 = @(t,y) C_NBdot(:,3);

[t1,C_NB1] = ode45(DCMdot_func1, tspan, C_NB0(:,1));
[t2,C_NB2] = ode45(DCMdot_func2, tspan, C_NB0(:,2));
[t3,C_NB3] = ode45(DCMdot_func3, tspan, C_NB0(:,3));

C_NB = [C_NB1(end,:)',C_NB2(end,:)',C_NB3(end,:)'];
[phi,theta,psi] = DCM2Euler(C_NB');

ThetaVec1 = [phi,theta,psi]'