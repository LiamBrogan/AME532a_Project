%% HW 2 Problem 2
% 2) Link the above (one of the rotational kinematics blocks, the 
% translational kinematics, and the translational dynamics) together. Allow
% orientation from the rotational kinematics to drive the DCMs, 
% translational dynamics to drive the translational kinematics, and so 
% forth. This is the coupling in our system of ODEs. Provide the applicable
% initial conditions and inputs (forces, gravity vectors, etc).

clear
close all
clc
%% Given Conditions
% Set up integration parameters
t0 = 0; % Initial time
dt = 1; % Time step
tend = 1000; % End time for simulation
tcount = 1; % Loop counter
timeVec = t0:dt:tend; % Period of integration

Rearth = 6378137.0; % meters
Yaw = 0; Pitch = 0; Roll = 0; % radians

X_EE_BE0 = [Rearth;0;0];
V0_BB_BE = [150; 0; 0];
thetaVec0_BN = [Roll; Pitch; Yaw];

state0 = [X_EE_BE0', V0_BB_BE', thetaVec0_BN'];

odefunc = @(t,state) state_derivative(t,state);
StateVec(tcount,:) = state0;

for time = timeVec(1:end-1)
    tcount = tcount+1;
    tspan = [time time+dt];

    [t,state] = ode45(odefunc, tspan, state0);
    
    state0 = state(end,:);
    [ state0(7:9) ] = eulerCheck( state0(7:9) );
    state0(7:9)
    StateVec(tcount,:) = state0;

end

figure(1)
plot3(StateVec(:,1),StateVec(:,2),StateVec(:,3))
grid on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Vehicle flight path in ECEF coordinates')
legend('X EE B/E')
