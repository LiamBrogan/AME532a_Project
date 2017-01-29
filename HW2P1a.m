%% Homework 3
%  Problem 1-a)
close all
clear
clc

%% i) Integrate Xdot_EE_BE over 1000 seconds of time to give X_EE_BE
Rearth = 6378137.0; % meters
Yaw = 0; Pitch = 0; Roll = 0; % radians
Lat = 0; Long = 0; % radians

C_NB = DCM_NED2Body(Roll,Pitch,Yaw)';
C_EN = DCM_ECEF2NED(Lat,Long)';
C_EB = C_EN*C_NB;

V_EB_BE = [150; 0; 0]; % m/s
Xdot_EB_BE = V_EB_BE;  % m/s;

Xdot_EE_BE = VelTransDCM(C_EB,Xdot_EB_BE,[],[]);

tspan = [0 1000];
X_EE_BE0 = [Rearth;0;0];
Xdot = @(t,y) Xdot_EE_BE;

%% i)
[t,X_EE_BE] = ode45(Xdot, tspan, X_EE_BE0);
plot(t,X_EE_BE)

%% ii)
a = 6378137;
b = 6356752;
e = (a^2-b^2)^.5/a;
h = 0;
N = a;

dif = 1;
Conv = 1e-10;
count = 0;
PHI = (atan(X_EE_BE(:,3)./((X_EE_BE(:,1).^2+X_EE_BE(:,2).^2).^.5).*(1-e^2*N./(N+h))));

while dif > Conv
    count = count+1;
    N = a./(1-e^2*sin(PHI).^2).^.5;
    hpN = ((X_EE_BE(:,1).^2+X_EE_BE(:,2).^2).^.5)./cos(PHI);
    h = hpN - N;
    PHI1 = (atan(X_EE_BE(:,3)./((X_EE_BE(:,1).^2+X_EE_BE(:,2).^2).^.5).*(1-e^2*N./(N+h))));
    dif(count) = PHI1(end)-PHI(end);
    PHI = PHI1;
end

LatVec = PHI;
LongVec = atan2(X_EE_BE(:,2),X_EE_BE(:,1));
AltVec = h;

LLA = [LatVec*a, LongVec*a, -AltVec];
% NED = geodetic2ned(LatVec,LongVec,AltVec);
% NED = [ , , -LLA(:,3)];

for i = 1:length(AltVec)
    C_EN_Vec(:,:,i) = DCM_ECEF2NED(LatVec(i),LongVec(i))';
    
    X_EE_BE2(i,:) = TransDCM(LLA(i,:)',C_EN_Vec(:,:,i));
    
    
end
X_EE_BE3(:,1) = (N+AltVec).*cos(LatVec).*cos(LongVec);
X_EE_BE3(:,2) = (N+AltVec).*cos(LatVec).*sin(LongVec);
X_EE_BE3(:,3) = ((1-e^2).*N+AltVec).*sin(LatVec);

figure
plot(t,X_EE_BE2)
figure
plot(t,X_EE_BE3)

min(X_EE_BE-X_EE_BE3)
max(X_EE_BE-X_EE_BE3)

