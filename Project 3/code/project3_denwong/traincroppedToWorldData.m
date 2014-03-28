%traincroppedToWorldData

%Take traincropped.mat and run UKF over to generate world training data
%file so we don't have to train everytime.

%load('traincropped.mat')

function [worldData] = traincroppedToWorldData(traincropped,varargin)

disp('Filtering data and converting IMUdata to world frame velocities')

if varargin{1} == 'test'
    disp('Test data - IMU data only, no classification')
    worldData = struct('data','','filename','');
else
    disp('Training data with classifications')
    worldData = struct('labelsIdx','','data','','label','','filename','');
    worldData(:).labelsIdx = traincropped(:).labelsIdx;
    worldData(:).label = traincropped(:).label;
    
end

plotUKFw = false;
plotUKFrot = false;
plotAccelWorld = false;

for i = 1:length(traincropped)
    worldData(i).filename = traincropped(i).filename;
    disp(i)
    %% Display filename
    %strfile = strcat('DataType: ',traincropped(1).labelsIdx(traincropped(i).label),' File: ',traincropped(i).filename);
    %disp(strfile)
    %% Use UKF to estimate orientation of device
    % IMUData = [Ax;Ay;Az;Wx;Wy;Wz], 6 x n matrix of IMU data
    IMUData = traincropped(i).data(2:end,:);
    tI = traincropped(i).data(1,:)';
    
    %% Determine start and end idx:
    cutTime(1) = 0;
    cutTime(2) = tI(end);
    
    [~,startIdx] = min(abs(tI - cutTime(1)));
    [~,endIdx] = min(abs(tI - cutTime(2)));
    
    stateIMU = applyUKF(IMUData(:,startIdx:endIdx),tI(startIdx:endIdx));
    
    if plotUKFw
        figure
        xlabelstr = 'time';
        ylabelstr = '\omega (rad/s)';
        
        % ang vel
        subplot(2,1,1)
        %dataset2plot(tI(startIdx:endIdx),stateIMU(5:end,:),xlabelstr,ylabelstr,'Gyro in World Frame')
        dataset2plot(tI(startIdx:endIdx),stateIMU(5:end,:),xlabelstr,ylabelstr,'Gyro in World Frame')
        title(strfile)
        legend('X','Y','Z')
        plot(tI(startIdx:endIdx),IMUData(4,startIdx:endIdx),'-k')
        plot(tI(startIdx:endIdx),IMUData(5,startIdx:endIdx),'-k')
        plot(tI(startIdx:endIdx),IMUData(6,startIdx:endIdx),'-k')
        
        %         subplot(2,1,2)
        %         hold on
        %         plot(tI(startIdx:endIdx),stateIMU(7,:),'.b')
        %         plot(tI(startIdx:endIdx),IMUData(6,startIdx:endIdx),'-k')
        %         title('Angular Velocity')
        %         xlabel('time (s)')
        %         ylabel('\omega_z (rad/s)')
        %         legend('estimate','IMU data')
        
        disp('MainProj3: ang vel plot finished')
        keyboard
    end
    
    %% plot orientation rotplot
    if plotUKFrot
        figure
        for m = startIdx:endIdx
            %subplot(1,2,1)
            %subplot(1,2,2)
            R = QuatToRot(stateIMU(1:4,m));
            rotplot(R,tI(m));
        end
    end
    
    %% Use UKF to estimate trajectory?
    state_world = zeros(6,size(IMUData,2));
    
    % rotate the AxAyAz from body frame to world frame
    for n = 1:length(stateIMU)-1
        imuIdx = n+startIdx-1;
        state_world(4:6,imuIdx) = QuatToRot(stateIMU(1:4,n))*IMUData(4:6,imuIdx); % rotate raw accel data from IMUData
        state_world(1:3,imuIdx) = QuatToRot(stateIMU(1:4,n))*stateIMU(5:end,imuIdx); % rotate w from stateIMU
    end
    
    if plotAccelWorld
        figure
        subplot(3,1,1)
        dataset2plot(tI(startIdx:endIdx),state_world(4:6,startIdx:endIdx),'time(s)','accel (m/s^2)','Accel in world frame')
        subplot(3,1,2)
        % velocity
        
        % posn
        keyboard
    end
    
   
    worldData(i).data = state_world;
    
    %keyboard
end
%save('worldData.mat','worldData')
