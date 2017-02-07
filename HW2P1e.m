%% HW 2 Problem 1-e) Translational Dynamics using a non-rotating earth
%  ie. Given Forces, gravity vector, your body rotational rates, and your 
%   velocity vector in the body coordinates at time=t0, provide your new 
%   velocity vector in the body coordinates at time=t0+dt.
close all
clear
clc
%% Initial Conditions
%  Given
F_BB = [1; 2; 3];
G_BB_BE = [1; 2; 3];
omega_BE = [1; 2; 3];
V0_BB_BE = [1; 2; 3];

% mass and time properties
m = 2;
t0 = 0;
dt = 0.001;

%% Vdot
Facc_BB = F_BB/m;

Vdot_BB_BE = VvecDot(Facc_BB,G_BB_BE,omega_BE,V0_BB_BE);

%% Translational Dynamics in non-rotating earth frame ECEF = ECI

tspan = [t0 t0+dt];
Vdot_func = @(t,y) Vdot_BB_BE;

[t,V_ode] = ode45(Vdot_func, tspan, V0_BB_BE);

V1_BB_BE = V_ode(end,:)';

function [Vdot_BE_BE] = VvecDot(Facc_BB,G_BB_BE,omega_BE,V_BB_BE)
%% HW 2 Problem 1-e) Translational Dynamics using a non-rotating earth
%  ie. Given Forces, gravity vector, your body rotational rates, and your 
%   velocity vector in the body coordinates at time=t0, provide your new 
%   velocity vector in the body coordinates at time=t0+dt.

Vdot_BE_BE = Facc_BB+G_BB_BE-cross(omega_BE,V_BB_BE);

end