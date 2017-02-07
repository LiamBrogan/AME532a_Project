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
tcount = 0; % Loop counter
timeVec = t0:dt:tend; % Period of integration

Rearth = 6378137.0; % meters
Yaw = 0; Pitch = 0; Roll = 0; % radians
Lat = 0; Long = 0; % radians
thetaVec0_BN = [Roll; Pitch; Yaw];
X_EE_BE0 = [Rearth;0;0];

F_BB0 = [2; 0; -9.807];
G_BB_BE0 = [0; 0; 9.807];
V0_BE_BE = [150; 0; 0];
omegaVec_BE = [0; 0.0000; 0.03];

m = 1; % kg

for time = timeVec(1:end-1)
    tcount = tcount+1;
    tspan = [time time+dt];
    %% Step 1 - Rotational kinematics used to find DCM

%     %  Thetavec0 and omegaVec are inputs to the Rotational kinematics step
%     [quat_BNdot,quat_BN0] = quatDot(thetaVec0_BN,omegaVec_BE);
%     quat_BN0
%     quat_BNdot
%     
%     quatDot_func = @(t,y) quat_BNdot;
%     [t,quat_BN] = ode45(quatDot_func, tspan, quat_BN0);
%     [phi,theta,psi] = Quat2Euler(quat_BN(end,:)');
% 
%     ThetaVec1 = [phi,theta,psi]'
%     thetaVec0_BN = ThetaVec1;
%     Theta(tcount,:) = ThetaVec1';

    ThetaDotVec = omegaDot2thetaDot( thetaVec0_BN, omegaVec_BE);
    ThetaDot = @(t,y) ThetaDotVec;

    [t,ThetaVec] = ode45(ThetaDot, tspan, thetaVec0_BN);
    
    ThetaVec1 = ThetaVec(end,:)';
    ThetaVec1 = eulerCheck( ThetaVec1 )'
    thetaVec0_BN = ThetaVec1
    
    Theta(tcount,:) = ThetaVec1';


    %% Step 2 - Translational dynamics used to find V

    Facc_BB = F_BB0/m;
    Vdot_BB_BE = VvecDot(Facc_BB,G_BB_BE0,omegaVec_BE,V0_BE_BE);
    Vdot_func = @(t,y) Vdot_BB_BE;
    [t,V_int] = ode45(Vdot_func, tspan, V0_BE_BE);
    V1_BE_BE(tcount,:) = V_int(end,:);
    V0_BE_BE = V1_BE_BE(tcount,:)';

    %% Step 3 - Translational kindematics from V used to find X

    C_NB = DCM_NED2Body(ThetaVec1(1),ThetaVec1(2),ThetaVec1(3))';
    C_EN = DCM_ECEF2NED(Lat,Long)';
    C_EB = C_EN*C_NB;

    Xdot_EE_BE = VelTransDCM(C_EB,V1_BE_BE(tcount,:)',X_EE_BE0,[]);

    Xdot = @(t,y) Xdot_EE_BE;

    [t,X_int] = ode45(Xdot, tspan, X_EE_BE0);
    X_EE_BE(tcount,:) = X_int(end,:);
    X_EE_BE0 = X_EE_BE(tcount,:)';
    
    a = 6378137;
    b = 6356752;
    e = (a^2-b^2)^.5/a;
    h = 0;
    N = a;
    
    % Set up convergence loop for latitude value
    dif = 1;
    Conv = 1e-10;
    count = 0;
    PHI = (atan(X_EE_BE(tcount,3)./((X_EE_BE(tcount,1).^2+X_EE_BE(tcount,2).^2).^.5).*(1-e^2*N./(N+h))));

    while dif > Conv
        count = count+1;
        N = a./(1-e^2*sin(PHI).^2).^.5;
        hpN = ((X_EE_BE(tcount,1).^2+X_EE_BE(tcount,2).^2).^.5)./cos(PHI);
        h = hpN - N;
        PHI1 = (atan(X_EE_BE(tcount,3)./((X_EE_BE(tcount,1).^2+X_EE_BE(tcount,2).^2).^.5).*(1-e^2*N./(N+h))));
        dif(count) = PHI1-PHI;
        PHI = PHI1;
    end
    
    % Use latitude integrated position values to get components of LLA
    % position
    LatVec = PHI;
    LongVec = atan2(X_EE_BE(tcount,2),X_EE_BE(tcount,1));
    AltVec = h;

    LLA = [LatVec*a, LongVec*a, AltVec];
    Lat = LatVec;
    Long = LongVec;
end

figure(1)
plot3(X_EE_BE(:,1),X_EE_BE(:,2),X_EE_BE(:,3))
grid on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Vehicle flight path in ECEF coordinates')
legend('X EE B/E_1')
