%% UKF test
%clear all
%close all
IMUData = [0 0 0 -pi/4 0 0]';
dt = 1e-2;
dt = .1;

%IMUData = [Ax;Ay;Az;Wx;Wy;Wz];
%dt = diff(tI);
%xkhat = zeros(7,size(IMUData,2));

dv = zeros(3,100);


xkhat(:,1) = UKF(IMUData(:,1),dt(1),'first');

w = xkhat(5:end);
Q = (xkhat(1:4));
R = QuatToRot(Q);
dz(1) = acos(R(1,1));
dv(:,1) = xkhat(5:end,1);
% Rgrdtruth = rotz(0.1)
% keyboard

%for i = 1:length(dt)
for i = 2:100
    % assuming perfect data:
    %disp(i);
    xkhat(:,i+1) = UKF(IMUData(:,1),dt(1));
    w = xkhat(5:end);
    Q = (xkhat(1:4));
    R = QuatToRot(Q);
    dz(i) = acos(R(1,1));
    dv(:,i) = xkhat(5:end,i+1);
end

 testfig = figure;
plot(dv(1,:),'.r')
hold on
plot(dv(2,:),'.g')
plot(dv(3,:),'.b')
ylabel('ang velocity (rad/s)')

% %% 
% for i = 875:2324
%     q = Quaternion(xkhat(1:4,i));
%     qRotation = q.R;
%     rotplot(qRotation,tI(i));
% end