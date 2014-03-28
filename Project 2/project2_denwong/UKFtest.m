%% UKF test
%clear all
%close all
IMUData = [0 0 0 -pi/4 0 0]';
dt = 1e-2;

%IMUData = [Ax;Ay;Az;Wx;Wy;Wz];
%dt = diff(tI);
%xkhat = zeros(7,size(IMUData,2));

xkhat(:,1) = UKF(IMUData(:,1),dt(1),'first');
% w = xkhat(5:end)
% Q = Quaternion(xkhat(1:4));
% R = Q.R
% dz(1) = acos(R(1,1));
% dv(:,1) = xkhat(5:end);

% Rgrdtruth = rotz(0.1)
% keyboard

%for i = 1:length(dt)
for i = 875:2324
    % assuming perfect data:
    %disp(i);
    xkhat(:,i+1) = UKF(IMUData(:,i+1),dt(i));
    %w = xkhat(5:end);
    %Q = Quaternion(xkhat(1:4));
    %R = Q.R;
    %dz(i) = acos(R(1,1));
    %dv(:,i) = xkhat(5:end);
end

testfig = figure;
%plot(dv(1,:),'.b')
%ylabel('ang velocity (rad/s)')
%% 
for i = 875:2324
    q = Quaternion(xkhat(1:4,i));
    qRotation = q.R;
    rotplot(qRotation,tI(i));
end