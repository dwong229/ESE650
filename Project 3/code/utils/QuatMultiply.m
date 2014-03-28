function [pq] = QuatMultiply(p,q)

% function to multiply quaternions in the form 
% p [4x1]: s,v1,v2,v3.
% q [4x1]: s,v1,v2,v3.
% pq [4x1]: s,v1,v2,v3.

pq = zeros(4,1);

p0 = p(1);
p1 = p(2);
p2 = p(3);
p3 = p(4);

q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

pq(1) = p0*q0 - p1*q1 - p2*q2 - p3*q3;
pq(2) = p0*q1 + p1*q0 + p2*q3 - p3*q2;
pq(3) = p0*q2 + p2*q0 - p1*q3 + p3*q1;
pq(4) = p0*q3 + p3*q0 + p1*q2 - p2*q1;

