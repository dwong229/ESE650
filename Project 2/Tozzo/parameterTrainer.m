%learn the IMU parameters from vicon data
%%
load('imuRaw1')
imuRaw = vals;
imuTime = ts;
clear ts vals
load('viconRot1')
vicon = rots;
viconTime = ts;
clear ts rots
%%
% load('vicon/viconRot1');
% time = ts - ts(1);
imuTime = imuTime - imuTime(1);
viconTime = viconTime - viconTime(1);
quatVector = zeros(4,size(vicon,3));
dthetaTrue = zeros(3,3, size(vicon,3)-1);
for i = 1:size(vicon,3)-1
    dthetaTrue(:,:,i) = (vicon(:,:, i+1) - vicon(:,:,i))/(viconTime(i+1) - viconTime(i));
    quatVector(:,i) = Quaternion(vicon(:,:,i)).double;
end
quatVector(:,i+1) = Quaternion(vicon(:,:,i+1)).double;
%%

dthetaRaw = zeros(6,size(imuRaw,2));
for i = 1:size(imuRaw,2)-1
    dthetaRaw(i) = (imuRaw(:,i+1) - imuRaw(:,i))/(imuTime(i+1) - imuTime(i));
end