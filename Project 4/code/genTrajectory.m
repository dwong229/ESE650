function [robotstate] = genTrajectory(edata,imudata)

% Return state of robot: time, world position and orientation

% INPUT
% edata: Encoder data: edata.ts = time[1 x n], edata.counts = [4 x n]
% imudata: IMU data: imudata.ts = time[1xm], imudata.vals = [6xm]

% OUTPUT
% robotstate: robotstate.vals = [6xp], robotstate.ts = [6xp]

% sensor modes
runUKF = true;
runOdom = true;

%% plot raw data
% figure
% subplot(2,1,1)
% plot(edata.ts,edata.counts(1,:),'-r',edata.ts,edata.counts(2,:),'-g',edata.ts,edata.counts(3,:),'-b',edata.ts,edata.counts(4,:),'-c')
% xlabel('time')
% ylabel('Encoder Counts')
% 
% subplot(2,1,2)
% hold all
% for iplot = 1:6
%     plot(imudata.ts,imudata.vals(iplot,:))
% end
% xlabel('time')
% ylabel('IMU values')

robotstate = [];

%% unpack data
startTime = min(edata.ts(1),imudata.ts(1));

tE = edata.ts - startTime;
encoder = edata.counts;

tI = imudata.ts - startTime;
[Ax,Ay,Az,Wx,Wy,Wz] = IMUfile2BodyFrame(imudata.vals);
IMUData = [Ax;Ay;Az;Wx;Wy;Wz];
%% Plot IMU data
% figure
% subplot(2,1,1)
% title('Accelerometer')
% plot(tI,Ax,'.r')
% hold on
% plot(tI,Ay,'.g')
% plot(tI,Az,'.b')
% xlabel('time')
% ylabel('m/s^2')
% 
% subplot(2,1,2)
% title('Gyro')
% plot(tI,Wx,'.r')
% hold on
% plot(tI,Wy,'.g')
% plot(tI,Wz,'.b')
% xlabel('time')
% ylabel('Angular Velocity')

%% Run UKF on IMU data to obtain orientation
if runUKF
    startIdx = 500;
    endIdx = length(tI);
    %endIdx = 1500;
    stateIMU = applyUKF(IMUData(:,startIdx:endIdx),tI(startIdx:endIdx));
    
    %% plot z angle from stateIMU
    rpy = zeros(3,size(stateIMU,2));
    for i = 1:size(stateIMU,2)
        rpy(:,i) = tr2rpy(QuatToRot(stateIMU(1:4,i)));
        
    end
    imuZangle = rpy(3,:);
    figure
    stateIMUlength = size(stateIMU,2);
    plot(1:stateIMUlength,rpy(1,:),'.b',1:stateIMUlength,rpy(2,:),'.g',1:stateIMUlength,rpy(3,:),'.c')
end

%% Convert Encoder info to body velocity
if runOdom
    robotstateencoder = zeros(3,length(tE));
    robotstateencoder = CountToDistance(edata.counts,tE);
    % = WheelDistanceToState(wheelDistance,edata.ts);
    
end
%% plot z angle from stateencoder
encoderZangle = robotstateencoder(3,:);
encoderZangle(encoderZangle>pi) = encoderZangle(encoderZangle>pi) - 2*pi;
figure
%plot(tE,rad2deg(encoderZangle(2:end)),'.g')
plot(tI(startIdx:endIdx),imuZangle,'.b',tE,encoderZangle(2:end),'.g')
keyboard
