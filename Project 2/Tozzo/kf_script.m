%UKF Script to estimate orientation
%% Load Data
load('imuRaw1')
imuRaw = vals;
imuTime = ts;
clear ts vals
load('viconRot1')
vicon = rots;
viconTime = ts;
clear ts rots
%% Setup and data conversion
Araw = [imuRaw(1:3,:); ones(1,length(imuTime))]';
plot(Araw(:,1:3));
% calMat = zeros(4,3);
% Anorm = Araw*calMat;
%apply rotation matrix from world to body frame
infRotMat = bsxfun(@minus, vicon, eye(3));
rpyVecTrue = zeros(size(infRotMat,3),3);
for i = 1:size(infRotMat, 3)
    rpyVecTrue(i,:) = vex(infRotMat(:,:,i));
end
accTrue = [-(rpyVecTrue(:,1)) (rpyVecTrue(:,2)) (rpyVecTrue(:,3))];
figure(); plot(accTrue);
% pitch = atan2(Ax, sqrt(Ay^2 + Az^2));
%roll = atan2(Ay, sqrt(Ax^2 + Az^2));
%%
biasAcc = 500; %from initial value of camera at rest
sensAcc = 6048; %(nominal from data sheet = 16384, 8192, 4096, 2048 LBS/g)
biasGyr = 372; %initial value of camera at rest
sensGyr = 64; %(nominal from data sheet = 131, 65.5, 32.8, 16.4 LBS/deg/sec)
deg2rad = pi/180;
Vref = 3300; %mV
maxADcon = 1;
accConvert = Vref/maxADcon * deg2rad / sensAcc;
gyrConvert = Vref/maxADcon * deg2rad / sensGyr;

%%
%vals array = [Ax Ay Az Wz Wx Wy]
accRaw = imuRaw(1:3,:);
accPhys = (accRaw - biasAcc) * accConvert;
gyrRaw = imuRaw(4:6,:);
gyrPhys = (gyrRaw - biasGyr) * gyrConvert;
figure()
plot(imuTime, accPhys(1,:), imuTime, accPhys(2,:), imuTime, accPhys(3,:))
legend('Ax', 'Ay', 'Az');
figure()
plot(imuTime, gyrPhys(2,:), imuTime, gyrPhys(3,:), imuTime, gyrPhys(1,:))
legend('Wx', 'Wy', 'Wz')
xState =[accPhys; gyrPhys];

%convert imu data to angles
