% 2)  For project, create the following functions/methods/scripts/blocks in
% C/C++/C#/Java/Perl/Python/Matlab/Simulink:
% a. Rotational Dynamics
% ie. Given Moments, an inertia tensor, and your body rotational rates in 
% body coordinates at time=t0, provide your new body rotational rates in 
% body coordinates at time=t0+dt.
close all
clear
clc

t0 = 0;
dt = 0.001;
tspan = [t0 t0+dt];

M_B_ext = [1;2;3];
Jmat = [3 0 0;
        0 2 0;
        0 0 6];
omega_BB_BI = [0;1;2];

[t,omega_BB_BI] = ode45(@OmegaDot, tspan, M_B_ext, [], Jmat, omega_BB_BI)

