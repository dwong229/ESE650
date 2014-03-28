%% Implement UKF

function [xkhat] = UKF(IMUmeasurements,dt,varargin)

% Given IMU data, compute state vector using UKF
% dt: Time between last measurement(k-1) and current input measurement(k)
% If varargin exists, measurements are just beginning
%% Set persistent values which will be carried between function calls.
persistent Pklast xkhatlast 
%% Input sensor measurements
%zk1 = [0 0 0]'; % angular velocity
%zk2 = [0 0 0]'; % linear acceleration

zk1 = IMUmeasurements(4:end);
zk2 = IMUmeasurements(1:3);

gyromeasurementupdate = true;
accelmeasurementupdate = false;
%% Initialize 
% if this is the first call
if nargin > 2
    disp('First iteration of UKF, initialize xkhat and Pk')
    % state vector
    %xkhatlast = [1; zeros(6,1)];
    xkhatlast = [1 0 0 0 0 0 0]';
    Pklast = eye(6); % dimension of measurements
end

%% sanity check: set all noise to zero
%R1 = eye(3)*0.00000076; % gyro noise from datasheet
%R1 = eye(3)*0.00000076; % gyro noise from datasheet
R1 = eye(3)*0.00000001; % gyro noise from datasheet
R2 = eye(3)*0.0001; % accelerometer noise
%R2 = zeros(3); % accelerometer noise
%Q = eye(6) * 0.001;
Q = eye(6)*0.001;
%Pklast = eye(6);
%% 1. Generate sigma points

% compute cholesky for P = S'S
if any(isnan(Pklast(:)))
    disp('NAN in Pklast')
    Pklast = eye(6);
end

eigs = eig(Pklast + Q);
if sum(eigs<0) > 0
    disp('P: not psd, reset Pklast')
    Pklast = eye(6);
end
    
S = chol(Pklast + Q);

% sigmapoints:
N = size(Q,2); % dimensions of statevector
x_sigmapoints = zeros(length(xkhatlast),2*N); % 7-dimension, 2n-sigma points
% sigma points TUNING PARAMETERS %%%%%%%%%%%%%%%%%%%%%
kappa = 0; % kappa>=0
alpha = 0.1; % 0<=alpha<=1 size of sigma points distribution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
lambda = alpha^2*(N + kappa) - N;
gamma = sqrt(N+lambda); % scaling factor

%% 2. Transform to 2N, -7-dimensional state vectors (sigma points)
for i = 1:N
    % convert first 3 components to quaterion
    W = gamma*S(:,i)*dt; % rad/time or rad/s/time * dt [6-dimensional vector]
    Rplus = rotx(W(1))*roty(W(2))*rotz(W(3));
    Rminus = rotx(-W(1))*roty(-W(2))*rotz(-W(3));
    % quaternion from last state
    qk = xkhatlast(1:4);
    % disturbed quaternion
    qwplus = RotToQuat(Rplus);
    qwminus = RotToQuat(Rminus);
    
    % New state
    newqplus = QuatMultiply(qk,qwplus);
    newqminus = QuatMultiply(qk,qwminus);
    x_sigmapoints(1:4,2*i - 1) = newqminus;
    x_sigmapoints(1:4,2*i) = newqplus;
    % add angular velocity
    x_sigmapoints(5:end,2*i - 1) = xkhatlast(5:end) - W(4:end);
    x_sigmapoints(5:end,2*i) = xkhatlast(5:end) + W(4:end);
    
end

%% 3. Project points Y = A(X) using process model
Yi = x_sigmapoints;
% apply angular velocity to quaternion
% angular velocity stays the same from sigma points because of assumed
% constant velocity
for i = 1:size(x_sigmapoints,2)
    w = x_sigmapoints(5:end,i)*dt;
    Rw = rotxyz(w(1),w(2),w(3));
    yq = QuatMultiply(RotToQuat(Rw),x_sigmapoints(1:4,i));
    Yi(1:4,i) = yq;
end

%% what the mean transformed is
% for troubleshooting
%xkhatmean = xkhatlast;
%w = xkhatlast(5:end)*dt;
%Rw = rotxyz(w(1),w(2),w(3));
%xkhatmeanq = Quaternion(Rw)*qk;
%xkhatmean(1:4) = xkhatmeanq;


%% 4. Compute a priori estimate xhatminus
% initialize xhatminus
xhatminus = zeros(size(xkhatlast));

% compute avg for all quaternions
q1 = qk; % start error calc from xkhatlast quaternion
t = 1;
smalle = false;
while t<21 && ~smalle
    eVec = zeros(3,size(Yi,2));
    for i = 1:size(Yi,2)
        q2 = [Yi(1:4,i)];
        q12 = QuatMultiply(q2,QuatInverse(q1)); %e-quaternion: rotation between mean and sigmapts
        th = acos(q12(1))*2;
        eVec(:,i) = q12(2:4)/sin(th/2)*th; %use last iteration of this for W (covariance)
    end
    % compute mean of e:
    emean = mean(eVec,2);
    % convert e to quaternion
    eth = norm(emean);
    emeanqs = cos(eth/2);
    if norm(emean)>0
        emeanqv = emean/norm(emean)*sin(eth/2);
    else
        emeanqv = [0 0 0]';    
    end  
    
    emeanquaternion = [emeanqs emeanqv'];
    % normalize e-quaternion
    %emeanquaternion = emeanquaternion/emeanquaternion.norm;
    
    qMean = QuatMultiply(emeanquaternion,q1);
    
    
    
    q1 = qMean;
    t = t +1; % counter
    
    if eth < 1e-12
        smalle = true;
        %fprintf('Small eth after %2.0f iterations. \n',t)
        
    end
    
    if ~isreal(q1)
    disp('Quaternion not real')
    keyboard
end

    
end


xhatminus(1:4) = q1;
% mean for angular velocity
xhatminus(5:end) = mean(Yi(5:end,:),2);

% generate a xkhat that will be updated in measurement step
xkhat = xhatminus;

if gyromeasurementupdate 
%% 5. Measurement Model:
omega_W = bsxfun(@minus, Yi(5:7,:),xhatminus(5:end));
thetas = zeros(1, size(eVec,2));
% r_W = zeros(4,size(eVec,2));
% for i = 1:size(eVec,2)
%     thetas(i) = norm(eVec(:,i));
%     r_W(1,i) = cos(thetas(i)/2);
%     r_W(2:4,i) = eVec(:,i)*sin(thetas(i)/2);
% end
% W_i = [r_W; omega_W];
W_i = [eVec; omega_W];
%% 6. Covariance Update
Pk_minus = zeros(size(W_i,1));
for i = 1:size(W_i,2)
    Pk_minus = Pk_minus + W_i(:,i)*W_i(:,i)';
end
Pk_minus = Pk_minus/size(W_i,2);
%% 7. Measurement Model Projection (7D to 3D)
% measurement model H1: angular velocity

% projected measurement vectors
Z_fromY = Yi(5:end,:);
zk_minus = mean(Z_fromY,2);
Pzz = zeros(3);
for i = 1:size(Z_fromY,2)
    Pzz = Pzz + (Z_fromY(:,i) - zk_minus)*(Z_fromY(:,i) - zk_minus)';
end
Pzz = Pzz/size(Yi,2);

%% 8. Compute innovation: compare actual measurement to zk_minus
vk = zk1 - zk_minus;

%% 9. Innovation covariance Pvv
Pvv = Pzz + R1;

%% 10. Cross correlation matrix Pxz from {Wi'} and {Zi'}
Pxz = zeros(3);
for i = 1:size(W_i,2)
    Pxz = Pxz + W_i(4:end,i)*(Z_fromY(:,i) - zk_minus)';
end
Pxz = Pxz/size(Yi,2);

%% 11. Update step: Compute xk and Pk using Kalman gain
Kk = Pxz*inv(Pvv); % Kalman gain

xkhat(5:end) = xkhat(5:end) + Kk*vk; % a posteriori estimate

% store persistent vectors
xkhatlast = xkhat;
Pklast = Pk_minus;
Pklast(4:end,4:end) = Pklast(4:end,4:end) - Kk*Pvv*Kk'; % estimate error covariance
% End gyro update

%% %%%%%%%%%%%% Accelerometer udpate %%%%%%%%%%%%%%%%%%%%%%
if accelmeasurementupdate
%% 7. Measurement Model Projection (7D to 3D)
% measurement model H2: orientation
%g = qvqQuaternion(Yi(1:4,:),[0 0 -1]');

%find the mean of g: 
Z_fromY = qvqQuaternion(Yi(1:4,:),[0 0 -9.81]');
zk_minus = mean(Z_fromY,2);
Pzz = zeros(3);
for i = 1:size(Z_fromY,2)
    Pzz = Pzz + (Z_fromY(:,i) - zk_minus)*(Z_fromY(:,i) - zk_minus)';
end
Pzz = Pzz/size(Yi,2);

%% 8. Compute innovation: compare actual measurement to zk_minus
vk = zk2 - zk_minus;

%% 9. Innovation covariance Pvv
Pvv = Pzz + R2;

%% 10. Cross correlation matrix Pxz from {Wi'} and {Zi'}
Pxz = zeros(3);
for i = 1:size(W_i,2)
    Pxz = Pxz + W_i(1:3,i)*(Z_fromY(:,i) - zk_minus)';
end
Pxz = Pxz/size(Yi,2);

%% 11. Update step: Compute xk and Pk using Kalman gain
Kk = Pxz*inv(Pvv); % Kalman gain

qupdaterotvec = Kk*vk;
alpha = norm(qupdaterotvec);
axis = qupdaterotvec'/norm(qupdaterotvec);
% convert rotation vector to quaternion
qupdate = [cos(alpha/2) axis*sin(alpha/2)];

% apply this rotation to xkhat quaternion
qnew = QuatMultiply(qupdate,xkhat(1:4));

xkhat(1:4) = qnew; % a posteriori estimate
end
else % save Pk
    disp('Not using gryo measurement update')
    keyboard
    xkhatlast = xkhat;
end