function [croppeddata] = cropData(data)

% plot and then take use input to crop data

IMUData = data(2:end,:);
tI = data(1,:);

%% plot data
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

croppeddata = [tIcrop; IMUData(:,startIdx:end)];
keyboard