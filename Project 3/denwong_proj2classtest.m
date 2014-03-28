%% Class test
close all
clear all
clc

testset = [10 11 12 13];

for k = 1:length(testset)

imuRawDataFile = strcat('imuRaw',num2str(testset(k)),'.mat');
camFileName = strcat('cam',num2str(testset(k)),'.mat');

% if ground truth exists, put filename here:
if k == 1;
    viconDataFile = 'viconRot10.mat';
else
    viconDataFile = '';
end

%% load IMUdatafile

load(imuRawDataFile)
[Ax,Ay,Az,Wx,Wy,Wz,TimeIMU] = IMUfile2BodyFrame(imuRawDataFile);
IMUData = [Ax;Ay;Az;Wx;Wy;Wz];

%% load camera data
disp('Camera File Found. Loading camera images.')
disp(camFileName)
[camData,TimeCam] = Cam2Data(camFileName);
       

%% Sync IMU and camera time:
MinStartTime = min([TimeCam(1),TimeIMU(1)]);

tI = TimeIMU - MinStartTime;
tC = TimeCam - MinStartTime;

% 
% if true
%     figure
%     for j = 1:5:size(camData,4)
%         imshow(camData(:,:,:,j))
%         title(num2str(tC(j)))
%         pause
%     end
% end
%% Run UKF
stateIMU = zeros(7,length(tI));
[stateIMU, activeTime] = applyUKF(IMUData,tI);

%% Convert all quaternions to rotation matrices
disp('Convert stateIMU quaternion to rotation matrices')
IMUrots = zeros(3,3,size(stateIMU,2));
for i = 1:size(stateIMU,2)
    %q = Quaternion(stateIMU(1:4,i));
    IMUrots(:,:,i) = QuatToRot(stateIMU(1:4,i));
end

%% Display estimated orientation
% show orientation and camera image in right frame
minFrame = 1;
maxFrame = length(TimeIMU);
hIMUCam = figure('Position',[443 202 1461 565]);
subplot(1,2,1)
hIMU = rotplot(IMUrots(:,:,1),tI(1));

subplot(1,2,2)
hCam = imshow(camData(:,:,:,1));
hCamTitle = title(sprintf('Camera Data \n Time: %5.3f',tC(1)));
uicontrol('Style', 'slider',...
    'Min',1,'Max',maxFrame,'Value',1,...
    'Position', [700 20 240 20],...
    'Callback', {@ViconCamSlider_Callback,hIMUCam,IMUrots,tI,hCam,hCamTitle,tC,camData});
uicontrol('Style','text',...
    'Position',[700 45 240 20],...
    'String','Select Time')

%% Compare to Vicon
if ~isempty(viconDataFile)
    disp('Compare UKF approximates to Vicon rotations')
    disp('Loading Vicon Data')
    load(viconDataFile)
    TimeVicon = ts;
    RVicon = rots;
    TimeViconStart = TimeVicon(1);
    tV = TimeVicon - MinStartTime;
    
    [orientationVicon,orientationIMU] = compareEulerAngles(tV,RVicon,tI,IMUrots); %uses Euler angle comparison
    
    [errorangle] = compareAxisAngle(tV,RVicon,tI,IMUrots);
    
    % plot orientation with error.
    subplot(2,1,2)
    dataset2plot(tI,orientationIMU,'time','Acceleration m/s','Accelerometer Data')
     
else
    disp('No Vicon datafile entered. Cannot compare to ground truth.')
end

%% Display Real-Time Panoramic image

%[panoImg] = genPano(tI,IMUrots,tC,camData);
%time = [9,25] % 8
if isempty(activeTime)
    activeTime = [1 tI(end)];
end
time = activeTime;
[~,idxC(1)] = min(abs(tC-time(1)));
[~,idxC(2)] = min(abs(tC-time(2)));

img = genPano(tI,IMUrots,tC(idxC(1):idxC(2)),camData(:,:,:,idxC(1):idxC(2)));
img = uint8(img);
figure
imshow(img)
keyboard
disp('End of one simualtion')

end

