function [xHat, Pk] = unscentedKF(xHatMinus, PkMinus, Q, R, dt, zK)
%unscented Kalman filter for orientation

%Steps 1-6 are the prediction phase
%% Step 1: Transform previous covariances into a set {W} of 2n 6D vectors distributed around 0 w/ Pk-1 +Q covar
% Q is process noise and is static
S = chol(PkMinus + Q);
n = size(PkMinus,1);
W = zeros(length(xHatMinus), 2*n); % W is the Sigma Delta to form the Sigma points
gamma = sqrt(2*n); %can adjust this to be sqrt(N + lambda) where lambda = alpha^2 *(N + k) - N
% alpha = [0,1]; k >= 0

% may need a dt factor to account for rad/sec units
W(:,1:n) = gamma*S*dt;
W(:,n+1:end) = -gamma*S*dt;
% W is 6x12 w/ rows 1-3 in angle form (x-y-z); the angles must be converted
% to quaternion form before forming the sigma points
% for i = 1:n
%     W(:,i) = gamma*S(:,i);
%     W(:,i+n) = -gamma*S(:,i);
%     
% end
%% Step 2: Apply previous state estimate to {W} to get set {X_i}, a 2n set of
% 7D state vectors called sigma points (that estimate the new probability distribution
qXminus = Quaternion(xHatMinus(1:4));
omegaXminus = xHatMinus(5:end);
SigmaPts = zeros(length(xHatMinus), 2*n);
for i = 1:2*n
    wQuat = Quaternion(rotx(W(1,i))*roty(W(2,i))*rotz(W(3,i)));
    temp = qXminus*wQuat;
    SigmaPts(1:4,i) = temp.double;
    SigmaPts(5:end,i) = omegaXminus + W(5:end,i);
end
%% Step 3: Apply the process model Y_i = A(X_i, 0)
Y = zeros(size(SigmaPts));
for i = 1:2*n
    eVec = SigmaPts(5:end,i);
    qDelta = Quaternion(rotx(eVec(1))*roty(eVec(2))*rotz(eVec(3)));
    qNext = Quaternion(SigmaPts(1:4,i))*qDelta;
    Y(1:4,i) = qNext.double;
    Y(5:end,i) = SigmaPts(5:end,i);
end
%% Step 4: Calculate the mean xHatMinus_k as the mean of the sigma points {Y_i}
% this is an iterative process
% use qXminus again as the start value
qMean = qXminus;
eMean = 1;
epsilon = 0.02;
t = 1;
while norm(eMean) > epsilon && t < 20
    eVec = zeros(3, size(Y,2));
    for i = 1:size(Y,2)
        q2 = Quaternion(Y(1:4,i));
        q12 = q2*qMean.inv; %e-quaternion: rotation between mean and sigmapts
        th = acos(q12.s)*2;
        eVec(:,i) = q12.v/sin(th/2)*th; %use last iteration of this for W (covariance)
    end
    eMean = mean(eVec,2);
    alpha = norm(eMean);
    eMeanScalar = cos(alpha/2);
    if norm(eMean)>0
        eMeanVec = eMean/norm(eMean)*sin(alpha/2);
    else % if eMean contains imaginary values
        eMeanVec = [0 0 0]';    
    end  
    eQuat = Quaternion([eMeanScalar eMeanVec]);
    qMean = eQuat*qMean;
    t = t + 1; %caps the number of iterations to 20 - seems to converge in that time anyways
end
xHatMinus_k = zeros(size(xHatMinus));
xHatMinus_k(1:4) = qMean.double;
xHatMinus_k(5:end) = mean(Y(5:end),2); % it's simple enough to take the mean of the velocities; quaternions are not in a vector space
%% Step 5: Transform set {Y_i} to set {W'} by removing the mean xHatMinus_k and convert it to quaternion
omegaPrime = bsxfun(@minus, Y(5:end,:), xHatMinus_k(5:end));
Wprime = [eVec; omegaPrime];

%% Step 6: Compute Pminus_k from {W'} using final iteration of mean computation from Step (5)
PkMinus = zeros(size(Wprime));
for i = 1:2*n
    PkMinus = PkMinus + Wprime(:,i)*Wprime(:,i)';
end
PkMinus = PkMinus/(2*n);
%Steps 7-11 are the measurement phase
%% Step 7: Apply the measurement model to project sigma points {Y_i} to get {Z_i}
% for the angular velocities, we can use mean() to average them
% however, for the orientation, we have to use quaternion multiplication
% and then convert them to rotation angles and THEN average them w/ mean
% zKminus is the mean value of the projected sigma points processed by the
% measurement function
zKminus = zeros(6,1);
zKminus(5:end) = mean(Y(4:end,:),2); % mean of the angular velocities

% qZmeasure is a temporary variable to hold the quaternion values before
% converting them to a quaternion, then back to rotation angles
qZmeasure = Y(1:4,:);
gravity = Quaternion([0 0 0 -9.81]); %gravity vector quaternion
qZhat = zeros(3, size(Y,2)); % quaternions projected over the gravity vector
for i = 1:size(qZmeasure,2)
    qTemp = Quaternion(qZmeasure(:,i));
    q = qTemp*gravity*qTemp.inv;
    qZhat(:,i) = q.v;
end
zKminus(1:3) = mean(qZhat,2); % mean of the projected quaternions

ZsigmaProj = [qZhat; Y(4:end,:)]; % create ZsigmaProj from Y - this is 6xn 

Pzz = zeros(size(ZsigmaProj));
for i = 1:size(ZsigmaProj,2)
    
end
%% Step 8: Compute the mean of {Z_i} (using similar method to step 5?) to get the measurement estimate, zMinus_k
%compare to actual measurement, z_k; v_k = z_k - zMinus_k where v_k is the
%innovation
vK = zK - zKminus; % vK is the innovation

%% Step 9: Innovation covariance, Pvv is determined from Pzz + R (static measurement covariance)
%Pzz comves from the covariance of Z_i: SUM(Z_i - zMinus_k * Z_i -
%zMinus_k')


%% Step 10: Pxz is computed from {W} and {Z_i - zMinus_k}

%% Step 11: Kalman gain, K_k is computed: K_k = Pxz*inv(Pvv) and then 
% used to calculate the a posteriori estimate xHat and its estimate error
% covar, Pk

end