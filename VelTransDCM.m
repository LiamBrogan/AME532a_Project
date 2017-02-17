function Vel_b = VelTransDCM(DCM,Vel_ab,Pos_ab,omega)
% Translate a velocity from coordinate system a to coordinate system b

if isempty(Pos_ab)
    Pos_ab = [0;0;0];
%     fprint('Default position set to [0;0;0]')
end

if isempty(omega)
    omega = [0;0;0];
%     fprint('Default rotation rate set to [0;0;0]')
end

Vel_b = TransDCM(Vel_ab,DCM)+cross(omega,TransDCM(Pos_ab,DCM));

end