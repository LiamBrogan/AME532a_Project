function [Vdot_BE_BE] = VvecDot(Facc_BB,G_BB_BE,omega_BE,V_BB_BE)
%% HW 2 Problem 1-e) Translational Dynamics using a non-rotating earth
%  ie. Given Forces, gravity vector, your body rotational rates, and your 
%   velocity vector in the body coordinates at time=t0, provide your new 
%   velocity vector in the body coordinates at time=t0+dt.

Vdot_BE_BE = Facc_BB+G_BB_BE-cross(omega_BE,V_BB_BE);

end

