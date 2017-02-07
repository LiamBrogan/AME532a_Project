%% Homework 3 
%%  Problem 1-a)Translation Kinematics (show plots of results for i and ii)
% Given a derivative taken in the Earth (ECEF) frame (ie inertial
% velocity), of the position vector of the vody wrt the Earth's Center,
% resolved in the body coordinate system (Xdot_EB_EB), and the orientation
% of the body wrt the Earth's center (C_EB = C_EN*C_NB), compute the
% derivative taken in the Earth frame, of the position vector of the body
% wrt the Earth's Center, resolved in the Earth's(ECEF coordinate system

close all
clear
clc
%% i) Integrate Xdot_EE_BE over 1000 seconds of time to give X_EE_BE
% Given
Rearth = 6378137.0; % meters
Yaw = 0; Pitch = 0; Roll = 0; % radians
Lat = 0; Long = 0; % radians

C_NB = DCM_NED2Body(Roll,Pitch,Yaw)';
C_EN = DCM_ECEF2NED(Lat,Long)';
C_EB = C_EN*C_NB;
C_EB2 = C_EB;

% Position and Velocity initial conditions
X_EE_BE0 = [Rearth;0;0]; % m
V_EB_BE = [150; 0; 0]; % m/s
Xdot_EB_BE = V_EB_BE;  % m/s;

% V initial condition in EE frame and coordinate system
Xdot_EE_BE = VelTransDCM(C_EB,Xdot_EB_BE,[],[]);

% Set up integration parameters
t0 = 0; % Initial time
dt = 1; % Time step
tend = 1000; % End time for simulation
tcount = 1; % Loop counter
timeVec = t0:dt:tend; % Period of integration

X_EE_BE_1(tcount,:) = X_EE_BE0;
X_EE_BE_2(tcount,:) = X_EE_BE_1(tcount,:);
X0_EE_BE_1 = X_EE_BE0;
X0_EE_BE_2 = X_EE_BE0;

for time = timeVec(1:end-1)
    tcount = tcount+1;
    tspan = [time time+dt];

    Xdot = @(t,y) Xdot_EE_BE;

    %% i) Integrate Xdot at each time step to get the X position
    [t,X_int] = ode45(Xdot, tspan, X0_EE_BE_1);
    X_EE_BE_1(tcount,:) = X_int(end,:);
    X0_EE_BE_1 = X_EE_BE_1(tcount,:);

    %% ii) Based on the current X_EE_BE_1 position, find the lat, long, alt
    %  coordinates.
    a = 6378137;
    b = 6356752;
    e = (a^2-b^2)^.5/a;
    h = 0;
    N = a;
    
    % Set up convergence loop for latitude value
    dif = 1;
    Conv = 1e-10;
    count = 0;
    PHI = (atan(X_EE_BE_1(tcount,3)./((X_EE_BE_1(tcount,1).^2+X_EE_BE_1(tcount,2).^2).^.5).*(1-e^2*N./(N+h))));

    while dif > Conv
        count = count+1;
        N = a./(1-e^2*sin(PHI).^2).^.5;
        hpN = ((X_EE_BE_1(tcount,1).^2+X_EE_BE_1(tcount,2).^2).^.5)./cos(PHI);
        h = hpN - N;
        PHI1 = (atan(X_EE_BE_1(tcount,3)./((X_EE_BE_1(tcount,1).^2+X_EE_BE_1(tcount,2).^2).^.5).*(1-e^2*N./(N+h))));
        dif(count) = PHI1-PHI;
        PHI = PHI1;
    end
    
    % Use latitude integrated position values to get components of LLA
    % position
    LatVec = PHI;
    LongVec = atan2(X_EE_BE_1(tcount,2),X_EE_BE_1(tcount,1));
    AltVec = h;

    LLA = [LatVec*a, LongVec*a, AltVec];
    
    % Find new DCM based on the LLA position
    C_EN2 = DCM_ECEF2NED(LatVec,LongVec)';
    C_EB2 = C_EN2*C_NB;
    
    % Integrate the next X2 position value based on the new rotation matrix
    Xdot_EE_BE2 = VelTransDCM(C_EB2,Xdot_EB_BE,X0_EE_BE_2,[]);
    Xdot2 = @(t,y) Xdot_EE_BE2;
    
    [t,X_int] = ode45(Xdot2, tspan, X0_EE_BE_2);
    X_EE_BE_2(tcount,:) = X_int(end,:);
    X0_EE_BE_2 = X_EE_BE_2(tcount,:)';
end

% Plot results
% X1 flight path
figure(1)
plot3(X_EE_BE_1(:,1),X_EE_BE_1(:,2),X_EE_BE_1(:,3))
grid on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Vehicle flight path in ECEF coordinates')
legend('X EE B/E_1')

% Both flight paths
figure(2)
plot3(X_EE_BE_1(:,1),X_EE_BE_1(:,2),X_EE_BE_1(:,3))
hold on
grid on
plot3(X_EE_BE_2(:,1),X_EE_BE_2(:,2),X_EE_BE_2(:,3))
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Vehicle flight path in ECEF coordinates')
legend('X EE B/E_1','X EE B/E_2')


