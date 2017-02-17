function [ omegaDot_BB_BI ] = OmegaDot(t,M_B_ext, Jmat, omega_BB_BI)
% 2)  For project, create the following functions/methods/scripts/blocks in
% C/C++/C#/Java/Perl/Python/Matlab/Simulink:
% a. Rotational Dynamics
% ie. Given Moments, an inertia tensor, and your body rotational rates in 
% body coordinates at time=t0, provide your new body rotational rates in 
% body coordinates at time=t0+dt.
OmegaMat = CrossMatrix(omega_BB_BI);
Jmat_inv = inv(Jmat);

omegaDot_BB_BI = Jmat_inv*(M_B_ext-OmegaMat*Jmat*omega_BB_BI);
end

