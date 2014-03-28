function [invq] = QuatInverse(q)

% compute a quaternion inverse of a quaternion

qconj = [q(1) -q(2) -q(3) -q(4)]';
invq = qconj/sum(q.*q);
