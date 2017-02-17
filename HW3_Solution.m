close all
clear
clc
% Script to demonstrate how ODE45() can be used to simulate translational
% kinematics, quaternion kinematics, and translational dynamics.
%initial position
x_E_E_BE_0 = [6.317e6; % X_E
0; % Y_E
0]; % Z_E
%initial attitude
theta_BN_0 = [0; %roll
pi/4; %pitch
0]; %yaw
q_BN_0 = euler2quat(theta_BN_0);
%initial velocity
v_B_B_BE_0 = [150; % U_B
0; % V_B
0]; % W_B
%initial body rates (assumed constant)
om_B_B_BE_0 = [0; %P
0; %Q
0.00]; %R
tSpan=[0:0.1:10]; %s
x_0 = [x_E_E_BE_0; % init position
q_BN_0; % init attitude
v_B_B_BE_0]; % init velocity
F_ext_B = [0;
0;
0];
mass = 10;
g_N = [0;
0;
9.81];
[t,x] = ode45(@transKinQuatKinTransDynOde, tSpan, x_0, [], om_B_B_BE_0, F_ext_B, mass,g_N);
figure(1)
hold on
plot(x(:,1),'--')
figure(2)
hold on
plot(x(:,2),'--')
figure(3)
hold on
plot(x(:,3),'--')
figure(4)
hold on
plot(x(:,4),'--')
figure(5)
hold on
plot(x(:,5),'--')
figure(6)
hold on
plot(x(:,6),'--')
figure(7)
hold on
plot(x(:,7),'--')
figure(8)
hold on
plot(x(:,8),'--')
figure(9)
hold on
plot(x(:,9),'--')
figure(10)
hold on
plot(x(:,10),'--')
r = sqrt(x(:,1).^2+x(:,2).^2+x(:,3).^2);
figure(11)
hold on
plot(r,'--')
function dxdt = transKinQuatKinTransDynOde(t,x,om_B_B_BE, F_ext_B, mass, g_N)
dxdt = zeros(length(x),1);
x_E_E_BE = x(1:3);
q_BN = x(4:7);
v_B_B_BE = x(8:10);
% TransKin
p_E_E_BE = x2p(x_E_E_BE);
C_EN = inv(latlon2dcm(p_E_E_BE(1), p_E_E_BE(2)));
theta_BN = quat2euler(q_BN);
C_NB = inv(euler2dcm(theta_BN(1), theta_BN(2), theta_BN(3)));
C_EB = C_EN * C_NB;
dxdt(1:3,1) = C_EB * v_B_B_BE;
% QuatKin
P = om_B_B_BE(1);
Q = om_B_B_BE(2);
R = om_B_B_BE(3);
QKM = [0 -P -Q -R;
P 0 R -Q;
Q -R 0 P;
R Q -P 0]; % Quaternion Kinematic Matrix
dxdt(4:7,1) = 0.5 * QKM * q_BN;
% TransDyn
dxdt(8:10,1) = F_ext_B/mass + C_NB'*g_N + cross(om_B_B_BE,v_B_B_BE);
end
function p = x2p(x)
r = norm(x);
lat = asin(x(3)/r);
lon = atan2(x(2),x(1));
p=[lat;
lon;
r];
end
function C_BN = euler2dcm(phi,theta,psi)
c1 = cos(psi);
c2 = cos(theta);
c3 = cos(phi);
s1 = sin(psi);
s2 = sin(theta);
s3 = sin(phi);
C_BN = [c1*c2 c2*s1 -s2;
-c3*s1+s3*s2*c1 c1*c3+s1*s2*s3 c2*s3;
s3*s1+c3*s2*c1 -c1*s3+s1*s2*c3 c2*c3];
end
function C_NE = latlon2dcm(lat,lon)
c1 = cos(lat);
c2 = cos(lon);
s1 = sin(lat);
s2 = sin(lon);
C1 = [ c1 0 s1;
0 1 0 ;
-s1 0 c1];
C2 = [ 0 0 1;
    0 1 0;
-1 0 0];
C3 = [ c2 s2 0;
-s2 c2 0 ;
0 0 1];
C_NE = C1*C2*C3;
end
function q = euler2quat(theta_vec)
phi = theta_vec(1);
theta = theta_vec(2);
psi = theta_vec(3);
c1 = cos(psi/2);
c2 = cos(theta/2);
c3 = cos(phi/2);
s1 = sin(psi/2);
s2 = sin(theta/2);
s3 = sin(phi/2);
q = [c3*c2*c1 + s3*s2*s1;
s3*c2*c1 - c3*s2*s1;
c3*s2*c1 + s3*c2*s1;
c3*c2*s1 - s3*s2*c1];
end
function theta_vec = quat2euler(quat)
phi = atan2( 2*(quat(1)*quat(2)+quat(3)*quat(4)), 1-2*(quat(2)^2+quat(3)^2) );
theta = asin ( 2*(quat(1)*quat(3)-quat(4)*quat(2)) );
psi = atan2( 2*(quat(1)*quat(4)+quat(2)*quat(3)), 1-2*(quat(3)^2+quat(4)^2) );
theta_vec = [phi;
theta;
psi];
end