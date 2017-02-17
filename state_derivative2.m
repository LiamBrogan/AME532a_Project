function stateDiv = state_derivative2(t,state,Flift_N,F_BB_BE,G_N,M_BB_BE,Jmat)
%% Set up constant conditions
% Constant conditions of earth
a = 6378137;
b = 6356752;
e = (a^2-b^2)^.5/a;
h = 0;
N = a;

%% Extract information from the current state
% Take state information
X_EE_BE = state(1:3);
V_BB_BE = state(4:6);
quat_BE = state(7:10);
omega_BE = state(11:13);

% Current Euler Angles
thetaVec_BN = Quat2Euler(quat_BE);

% Set up loop counters
dif = 1;
Conv = 1e-10;
count = 0;
lat = (atan(X_EE_BE(3)./((X_EE_BE(1).^2+X_EE_BE(2).^2).^.5).*(1-e^2*N./(N+h))));

% Loop to converge on solution for lat
while dif > Conv
    count = count+1;
    N = a./(1-e^2*sin(lat).^2).^.5;
    hpN = ((X_EE_BE(1).^2+X_EE_BE(2).^2).^.5)./cos(lat);
    h = hpN - N;
    lat1 = (atan(X_EE_BE(3)./((X_EE_BE(1).^2+X_EE_BE(2).^2).^.5).*(1-e^2*N./(N+h))));
    dif(count) = lat1-lat;
    lat = lat1;
end

% Find LLA for aircraft
Lat = lat;
Long = atan2(X_EE_BE(2),X_EE_BE(1));
Alt = h;

% Create necessary DCMs for transformations
C_NB = inv(DCM_NED2Body(thetaVec_BN));
C_BN = inv(C_NB);
C_EN = inv(DCM_ECEF2NED(Lat,Long));
C_EB = C_EN*C_NB;

% Create Gravity and Force vectors by rotating lift and gravity to body
% frame and coordinate system
G_BB_BE = C_BN*G_N;
Facc_BB_BE = F_BB_BE + C_BN*Flift_N;

% Calculate derivatives of the state
% thetaDot = omega2thetaDot( thetaVec_BN, omega_BE);
xDot_EE_BE = VelTransDCM(C_EB,V_BB_BE,X_EE_BE,[]);
velDot_BB_BE = VvecDot(Facc_BB_BE,G_BB_BE,omega_BE,V_BB_BE);
quatDot_BE = quatDot(thetaVec_BN,omega_BE);
omegaDot_BE = OmegaDot(t,M_BB_BE, Jmat, omega_BE);

% Output of function
stateDiv = [xDot_EE_BE;
            velDot_BB_BE;
            quatDot_BE;
            omegaDot_BE];

end