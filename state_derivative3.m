function stateDiv = state_derivative3(t,state,Flift_N,F_BB_BE,G_N,M_BB_BE,Jmat,h_B_int,hdot_B_int)
%% Set up constant conditions
% Constant conditions of earth
rearth = 6378137;

%% Extract information from the current state
% Take state information
X_EE_BE = state(1:3);
V_BB_BE = state(4:6);
quat_BE = state(7:10)/norm(state(7:10));
omega_BE = state(11:13);

% Current Euler Angles
thetaVec_BN = Quat2Euler(quat_BE);

% Find LLA for aircraft
rpos = norm(X_EE_BE);
Lat = asin(X_EE_BE(3)/rpos);
Long = atan2(X_EE_BE(2),X_EE_BE(1));
Alt = rpos-rearth;

% Create necessary DCMs for transformations
C_NB = inv(DCM_NED2Body(thetaVec_BN));
C_BN = inv(C_NB);
C_EN = inv(DCM_ECEF2NED(Lat,Long));
C_EB = C_EN*C_NB;

% Create Gravity and Force vectors by rotating lift and gravity to body
% frame and coordinate system
G_BB_BE = C_BN*G_N;
Facc_BB_BE = F_BB_BE + C_BN*Flift_N;

% Create Matrix for quatdot equation
p = omega_BE(1);
q = omega_BE(2);
r = omega_BE(3);
omegaQuatMat = [0 -p -q -r;
                p  0  r -q;
                q -r  0  p;
                r  q -p  0];

% Create matrices for omegaDot Equation
Jmat_inv = inv(Jmat);
OmegaMat = CrossMatrix(omega_BE);

% Calculate derivatives of the state
xDot_EE_BE = C_EB*V_BB_BE;
velDot_BB_BE = Facc_BB_BE+G_BB_BE+cross(omega_BE,V_BB_BE);
quatDot_BE = 1/2*omegaQuatMat*quat_BE;
omegaDot_BE = Jmat_inv*(M_BB_BE-OmegaMat*Jmat*omega_BE-hdot_B_int-OmegaMat*h_B_int);

% Output of function
stateDiv = [xDot_EE_BE;
            velDot_BB_BE;
            quatDot_BE;
            omegaDot_BE];

end