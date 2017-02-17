% 4) For project, add the following to your state equations in
% C/C++/C#/Java/Perl/Python/Matlab/Simulink:
% a. Internal components that have varying rotational momentum
% b. Computation of total speed, angle of attack, and side-slip angle (Vinf, alpha, beta)
% c. Convert the translational kinematics from rectangular (x) to spherical (p)

clear
close all
clc
%% Given Conditions
% Set up integration parameters
t0 = 0; % Initial time
dt = .1; % Time steptcount
tend = 10; % End time for simulation
tcount = 1; % Loop counter
timeVec = t0:dt:tend; % Period of integration

Rearth = 6378137.0; % meters
Yaw = pi/2; Pitch = pi/2.0000000; Roll = 0; % radians

X_EE_BE0 = [Rearth;0;0];
V0_BB_BE = [ 0; 4; 0];
thetaVec0_BN = [Roll; Pitch; Yaw];
quat0_BE = Euler2Quat(thetaVec0_BN);

F_BB_BE = [50; 0; 0];
Flift_N = [0; 0; -9.807];
G_N = [0; 0; 9.807];
omega_BE = [-.04; -0.15; .06];

M_B_ext = [1;2;-.3];
Jmat = [10 0 0;
        0 13 0;
        0 0 25];
    
h_B_int = [3;
           4;
           -1];
    
hdot_B_int = [0;
              0;
              0];

state0 = [X_EE_BE0', V0_BB_BE', quat0_BE', omega_BE'];

StateVec(tcount,:) = state0;
euler(tcount,:) = Quat2Euler(state0(7:10));

for time = timeVec(1:end-1)
    tcount = tcount+1;
    tspan = [time time+dt];

    [t,state] = ode45(@state_derivative3, tspan, state0, [], Flift_N, F_BB_BE, G_N, M_B_ext,Jmat, h_B_int, hdot_B_int);
    
    state0 = state(end,:);
    state0(7:10) = state0(7:10)/norm(state0(7:10));
    StateVec(tcount,:) = state0;
    euler(tcount,:) = Quat2Euler(state0(7:10));
end

% b. Computation of total speed, angle of attack, and side-slip angle (Vinf, alpha, beta)
Vinf = sqrt(StateVec(:,4).^2+StateVec(:,5).^2+StateVec(:,6).^2);
alpha = atan2(StateVec(:,6),StateVec(:,4));
beta = asin(StateVec(:,5)./Vinf);
gvec = [Vinf, alpha, beta];

% c. Convert the translational kinematics from rectangular (x) to spherical (p)
r = sqrt(StateVec(:,1).^2+StateVec(:,2).^2+StateVec(:,3).^2);
lat = asin(StateVec(:,3)./r);
long = atan2(StateVec(:,2),StateVec(:,1));
pvec = [lat, long, r];

% Problem 6
% Start working on visualization techiniques

figure(1)
plot3(StateVec(:,1),StateVec(:,2),StateVec(:,3))
grid on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Vehicle flight path in ECEF coordinates')
legend('X EE B/E')

figure(2)
plot(timeVec,euler(:,1),timeVec,euler(:,2),timeVec,euler(:,3))
grid on
xlabel('Time (s)')
ylabel('Euler Angle (rad)')
title('Vehicle Euler Angles')
legend('Roll','Pitch','Yaw')

figure(3)
plot(timeVec,StateVec(:,11),timeVec,StateVec(:,12),timeVec,StateVec(:,13))
grid on
xlabel('Time (s)')
ylabel('Euler Angle (rad)')
title('Vehicle Body Rates')
legend('Roll Rate','Pitch Rate','Yaw Rate')