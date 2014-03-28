function [g] = qvqQuaternion(quadMat,v)


% quadMat: 4xn matrix of quaternion.double.
% v: vector to rotate by quadMat 3x1


alpha = v(1);
beta = v(2);
gamma = v(3);


g = zeros(3,size(quadMat,2));

for i = 1:size(quadMat,2)
    q = quadMat(:,i);
    
    % unpack quaternion
    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);
    
    fqi = [q0^2 + q1^2 - q2^2 - q3^2;2*q0*q3 + 2*q1*q2;-2*q0*q2 + 2*q1*q3];
    fqj = [-2*q0*q3 + 2*q1*q2;q0^2 - q1^2 + q2^2 - q3^2;2*q0*q1 + 2*q2*q3];
    fqk = [2*q0*q2 + 2*q1*q3;-2*q0*q1 + 2*q2*q3;q0^2 - q1^2 - q2^2 + q3^2];
    
    g(:,i) = alpha*fqi + beta*fqj + gamma*fqk;
        
end


