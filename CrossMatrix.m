function [ CrossMat ] = CrossMatrix(Vector)
%When a matrix cross product is part of some matrix equation, it can be
%easier to multiply by a cross matrix.  This makes the equation more
%algebraic, and easier to compute.
v = Vector;

CrossMat = [0   -v(3) v(2);
            v(3) 0    v(1);
           -v(2) v(1) 0];

end

