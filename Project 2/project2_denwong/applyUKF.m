function [xkhat,activeTime] = applyUKF(IMUData,tI)

% Input
% IMUData = [Ax;Ay;Az;Wx;Wy;Wz], 6 x n matrix of IMU data
% tI, 1 x n matrix of time
%
% Output
% xkhat, 7xn, state vector 4x1 quaternion [s v], 3 x 1 [wx wy wz]

% convert time to time difference
dt = diff(tI);

xkhat = zeros(7,size(IMUData,2));

xkhat(:,1) = UKF(IMUData(:,1),dt(1),'first');

%% set limits 
textinput = input('Do you want enter active time limits, (y/n): \n','s');

if textinput == 'y'
    %activeTime = [15 20];
    activelimitinput = input('Enter active time limits, [#,#]:\n','s');
    activeTime = str2num(activelimitinput);
else
    disp('Evaluate entire dataset.')
    activeTime = [];
end

if isempty(activeTime)
    startIdx = 1;
    endIdx = length(dt);
else
    startIdx = find(tI>activeTime(1),1,'first');
    endIdx = find(tI<activeTime(2),1,'last');
end

%% run UKF
%for i = 1:length(dt)
for i = startIdx:endIdx
    xkhat(:,i+1) = UKF(IMUData(:,i+1),dt(i));
end

%% 
textinput = input('Do you want to watch rotation (y/n): \n','s');

if textinput == 'y'
    
    testfig = figure;
    for i = startIdx:endIdx
        q = Quaternion(xkhat(1:4,i));
        qRotation = q.R;
        rotplot(qRotation,tI(i));
    end
end