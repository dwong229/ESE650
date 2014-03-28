%% Project 2 ESE 650 Orientation Tracking
% Denise Wong
% 2/4/2014

close all
clear all
clc

%% Extract IMU data

% change these handles to run code on different datasets

imuRawDataFile = 'imuRaw11.mat'
viconDataFile = 'viconRot10.mat'

camDataFile = '';

%% plot options:
plotViconCameraGUI = true;
plotIMURawData = true;
plotIMUaWorld = false;
plotViconData = true;
plotViconCheckIntegration = false;
plotUKFw = false;
plotViconUKFw = true;

%%
load(imuRawDataFile)
% extracts time:ts, IMUpacket:vals
% ts : unix time
% vals = [Ax Ay Az Wz Wx Wy]', 10-bit ADC values
% Ax and Ay direction are flipped, positive accel will body frame will
% result in negative accel reported by IMU.

% Acc = value * scale factor
% N = V * N/V

Vref = 3300; %mV
accel_sensitivity = 1; %mV/g, Least Significant Bit per Degrees per second
gyro_sensitivity = 1;

accel_bias = 0;
gyro_bias = 0;

%accel_scale_factor = Vref/1023 * accel_sensitivity;

[Ax,Ay,Az,Wx,Wy,Wz,TimeIMU] = IMUfile2BodyFrame(imuRawDataFile);
%,accel_Vref,accel_sensitivity,gyro_Vref,);

%% Extract for Vicon data file

if ~isempty(viconDataFile)
    disp('Loading Vicon Data')
    load(viconDataFile)
    TimeVicon = ts;
    RVicon = rots;
%    hVicon = figure;
    TimeViconStart = TimeVicon(1);
    TimeDiff = TimeVicon - TimeViconStart;
else
    disp('No Vicon Data File')
end

%% CAMERA: Get camera data if file exists

tempFileName = strtok(imuRawDataFile,'.');
FileNum = tempFileName(end);

camFileName = cat(2,'cam',FileNum,'.mat');
camFileName = 'cam11.mat';

if exist(camFileName,'file')==2
    disp('Camera File Found. Loading camera images.')
    disp(camFileName)
    [camData,TimeCam] = Cam2Data(camFileName);
    %figure;
    %camh = imshow(camData(:,:,:,1));
    
    %for i = 1:length(TimeCam)
    %    set(camh,'CData',(camData(:,:,:,i)));
    %    pause(.01)
    %end
else
    disp('No cam file found')
end

%% Sync IMU, Vicon and Camera Time
%MinStartTime = min([TimeCam(1),TimeVicon(1),TimeIMU(1)]);
MinStartTime = min([TimeCam(1),TimeIMU(1)]);

tI = TimeIMU - MinStartTime;
tV = TimeVicon - MinStartTime;
tC = TimeCam - MinStartTime;
keyboard
%% Rotate Accelerometer data, should give angle of g:
%use Vicon data:
%dist = zeros(3,length(TimeIMU)); %[x y z] posn of camera in world frame
%vel = zeros(3,length(tI)); %[x y z] velocity of camera in world frame
gVec = zeros(3,length(TimeIMU)); % [x y z] vector of

% Integrate linear acceleration

for i = 1:length(TimeIMU)
    [~,viconIdx] = min(abs(tV - tI(i)));
    gVec(:,i) = rots(:,:,viconIdx) * [Ax(i);Ay(i);Az(i)];
end

%% create a vicon and camera data gui
if plotViconCameraGUI
    minFrame = 1;
    maxFrame = length(TimeVicon);
    hVicCam = figure('Position',[443 202 1461 565]);
    subplot(1,2,1)
    hVicon = rotplot(rots(:,:,1),tV(1));
    
    subplot(1,2,2)
    hCam = imshow(camData(:,:,:,1));
    hCamTitle = title(sprintf('Camera Data \n Time: %5.3f',tC(1)));
    uicontrol('Style', 'slider',...
        'Min',1,'Max',maxFrame,'Value',1,...
        'Position', [700 20 240 20],...
        'Callback', {@ViconCamSlider_Callback,hVicCam,rots,tV,hCam,hCamTitle,tC,camData,tI,gVec});
    uicontrol('Style','text',...
        'Position',[700 45 240 20],...
        'String','Select Time')
end
keyboard
%% plot Linear accelerometer data - adjusted
if plotIMURawData
    figure;
    fighandle = subplot(2,1,1);
    dataset2plot(tI,[Ax;Ay;Az],'time','Acceleration m/s','Accelerometer Data')
    subplot(2,1,2)
    dataset2plot(tI,[Wx;Wy;Wz],'time','Ang Vel rad/s','Gyro in Body Frame')
end

%% Plot accerlerometer data in world frame:
if plotIMUaWorld
    disp('Rotated gVec')
    hgVec = figure;
    dataset2plot(tI,gVec,'time','Acceleration m/s','World Frame accelerometer data')
end

%% Compute angular velocity from vicon
dtVicon = diff(TimeVicon);
angvelVicon = zeros(3,length(dtVicon));
rpyVicon = zeros(3,length(dtVicon)+1);
angvelViconFromrpy = zeros(3,length(dtVicon));


rpyVicon(:,1) = tr2rpy(RVicon(:,:,1));
for i = 1:length(dtVicon);
    rpyVicon(:,i+1) = tr2rpy(RVicon(:,:,i+1));
    
    angvelViconFromrpy(:,i) = (rpyVicon(:,i+1) - rpyVicon(:,i))/dtVicon(i);
        
    if dtVicon(i) >  0.001
        % Throw away really small delta times and set velocity to 0.
        dtS = (RVicon(:,:,i+1)*RVicon(:,:,i)' - eye(3));
        angvelVicon(:,i) = (vex(dtS)/dtVicon(i));
    end
    
    

    %S = 1/dtVicon(i)*(RVicon(:,:,i+1) - RVicon(:,:,i)) * RVicon(:,:,i)';
    %w1 = mean([-S(2,3), S(3,2)]);
    %w2 = mean([S(1,3), -S(3,1)]);
    %w3 = mean([-S(1,2), S(2,1)]);
    
    %disp('Compare rotation matrices')
    %RVicon(:,:,i+1)
    %skewsym = [0 -w3 w2;w3 0 -w1;-w2 w1 0];
    %Rfromskewsym = rotx(w1*dtVicon)*roty(w2*dtVicon)*rotz(w3*dtVicon); %pre mult with R<i> to get R<i+1>.
    %Rcheck = RVicon(:,:,i) + dtVicon(i)*skewsym*RVicon(:,:,i)
    
    % compute using XYZ convention
    %R2computed = rotx(angvelVicon(1,1)*dtVicon(i))*roty(angvelVicon(2,1)*dtVicon(i))*rotz(angvelVicon(3,1)*dtVicon(i))*RVicon(:,:,i);
    %R2 = RVicon(:,:,i+1)
end

if plotViconData
    figure
    subplot(2,1,1)
    plot(tV,rpyVicon(1,:),'.r')
    hold on
    plot(tV,rpyVicon(2,:),'.g')
    plot(tV,rpyVicon(3,:),'.b')
    subplot(2,1,2)
    plot(tV(1:end-1),angvelViconFromrpy(1,:),'.r')
    hold on
    plot(tV(1:end-1),angvelViconFromrpy(2,:),'.g')
    plot(tV(1:end-1),angvelViconFromrpy(3,:),'.b')
    
    
end

%% cycle through w from Vicon to see if it matched with real

if plotViconCheckIntegration
    %checwfromVicon(RVicon,tV,angvelVicon)
    checwfromViconGyro(tV,angvelVicon,tI,Wx,Wy,Wz)
end


%% Statevector from Vicon:
disp('Generate ground truth state vector from Vicon Data')
stateVicon = zeros(7,length(dtVicon)); % state time corresponds to tV
stateVicon(5:7,:) = angvelVicon;
for i = 1:length(dtVicon)
    tempQ = Quaternion(RVicon(:,:,i));
    stateVicon(1:4,i) = tempQ.double';
end
%% Process gyro data using UKF

% package IMUData for UKF
IMUData = [Ax;Ay;Az;Wx;Wy;Wz];

stateIMU = applyUKF(IMUData,tI);


%% Plot UKF derived states - angular velocity
keyboard
if plotUKFw
    figure
    fighandle = subplot(3,1,1)
    dataset2plot(fighandle,tIcut,stateIMU(5:end,:),xlabelstr,ylabelstr)
%     plot(tIcut,stateIMU(5,:),'xr','MarkerSize',5)
%     hold on
%     plot(tV(2:end),stateVicon(5,:),'.k','MarkerSize',2)
%     
%     subplot(3,1,2)
%     plot(tIcut,stateIMU(6,:),'xg','MarkerSize',5)
%     hold on
%     plot(tV(2:end),stateVicon(6,:),'.k','MarkerSize',2)
%     
%     subplot(3,1,3)
%     plot(tIcut,stateIMU(7,:),'xb','MarkerSize',5)
%     hold on
%     plot(tV(2:end),stateVicon(7,:),'.k','MarkerSize',2)
end

xlabel('Time')
ylabel('rad/sec')
title('Gyro in World Frame')
legend('X','Y','Z')

%% Ingrated gryo data
% th = integratew([Wx;Wy;Wz],diff(tI)); % integrated gyro data

%% compare orientation to actual orientation
if plotViconUKFw
    figure
    
    [~,startIdxtv] = min(abs(tV - cutTime(1)));
    [~,endIdxtv] = min(abs(tV - cutTime(2)));
    
    
    for i = startIdxtv:endIdxtv
        
        subplot(1,2,1)
        rotplot(RVicon(:,:,i+1),tV(i+1));
        
        [~,IMUTimeIdx] = min(abs(tIcut - tV(i)));
        subplot(1,2,2)
        q = Quaternion(stateIMU(1:4,IMUTimeIdx));
        qRotation = q.R;
        rotplot(qRotation,tIcut(IMUTimeIdx));
        
        % compare to integrated gyro data :
        %rotplot(rotxyz(th(1,i),th(2,i),th(3,i)),tI(IMUTimeIdx));
        
    end
end
