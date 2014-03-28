%% Crop data train -> traincropped

% cycle through each dataset and crop only the good patterns
load('trainingdata.mat')

traincropped = struct('labelsIdx','','data','','label','','filename','');
traincropped(:).labelsIdx = train(:).labelsIdx;
traincropped(:).label = train(:).label;
traincropped(:).filename = train(:).filename;

h1 = figure; %for ginput
set(h1,'Position',[955 156 560 420]);
for i = 1:30
    disp(i)
    %% Display filename
    strfile = strcat('DataType: ',train(1).labelsIdx(train(i).label),' File: ',train(i).filename);
    disp(strfile)
    %% Use UKF to estimate orientation of device
    % IMUData = [Ax;Ay;Az;Wx;Wy;Wz], 6 x n matrix of IMU data
    IMUData = train(i).data(:,2:end)';
    tI = train(i).data(:,1)';
    
    %% plot data
    figure(h1)
    subplot(2,1,1)
    dataset2plot(tI,IMUData(1:3,:),'time','IMUreading','RawIMUdata')
    dataset2plot(tI,IMUData(4:6,:),'time','IMUreading','RawIMUdata')
    hold off
    
    disp('Select start time for data')
    [startTime ~] = ginput(1)
    
    % deduce the starting index
    [~,startIdx] = min(abs(tI - startTime));
    
    subplot(2,1,2)
    tIcrop = tI(startIdx:end) - tI(startIdx);
    dataset2plot(tIcrop,IMUData(1:3,startIdx:end),'time','IMUreading','RawIMUdata')
    dataset2plot(tIcrop,IMUData(4:6,startIdx:end),'time','IMUreading','RawIMUdata')
    hold off
    
    traincropped(i).data = [tIcrop; IMUData(:,startIdx:end)];
    
    keyboard
end
save('traincropped.mat','traincropped')
