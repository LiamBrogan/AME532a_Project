function r = QuatMult(p,q)
% For project, create the following functions/methods/scripts/blocks in
% C/C++/C#/Java/Perl/Python/Matlab/Simulink:
% a. Quaternion multiplication
%    i. this is equivalent to coordinate transforms using quaternions
%    ii. Use vector dot and cross multiplication to aid in this
%    iii. If using C/C++ look into using the boost package to work with vectors/matrices

po = p(1);
pr = p(2:4);
qo = p(1);
qr = q(2:4);

r = [0 0 0 0]';
r(1) = (po*qo)-dot(po,qo);
r(2:4) = (po*qr)+(qo*pr)+cross(pr,qr);

end

