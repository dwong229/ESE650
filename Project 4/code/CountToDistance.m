function [qt] = CountToDistance(counts,time)

%% dead reckoning 

% Convert encoder counts to distance of wheel travelled
% INPUT
% counts: [4 x n], [FR FL RR RL]
% 
% OUTPUT
% cumulative distance travelled by each wheel at each time from start 
%

% encoder parameters FOR kinect robot!
CountsPerRev = 360; % counts per revolution of the wheel/360deg
WheelDiameter = 254; % mm
% robot geometry:
WheelAxis = 330.20; %mm (front wheel axis to back wheel axis)
WheelToWheelInner = 311.15; % mm (inner left wheel to inner right wheel)
WheelToWheelOuter = 476.25; % mm (outer left wheel to outer right wheel)


% initialize output
wheelDistances = zeros(2,size(counts,2));

% computation
mmPerRev = pi*WheelDiameter;
mmPerCount = mmPerRev/CountsPerRev; 
WheelToWheelAvg = (WheelToWheelInner+WheelToWheelOuter)/2*1.85;

%robotRadius = sqrt((WheelAxis/2)^2 + (WheelToWheelAvg/2)^2);


robotState = zeros(6,length(time)); %robotState: [X,Y,Z,thx,thy,thz]
% Z,thx and thy will always be zero since with wheel odom we are always
% assuming planar motion

% average encoder counts
LRcounts = [mean(counts([2,4],:),1);mean(counts([1,3],:),1)]; % average counts for left and right wheels [2 x n]

% convert encoder counts to distance travelled
wheelDistances = LRcounts*mmPerCount;



motionDir = zeros(1,length(time));

qt = zeros(3,length(time)+1);
%% cycle through the times:
for timeIdx = 1:length(time)
%     % Check to see if robot is moving forward or rotating
%     if LRcounts(1,timeIdx) == LRcounts(2,timeIdx)
%         % robot is moving straight
%         if LRcounts(1,timeIdx) > 0
%             % forward
%             motionDir(timeIdx) = 1;
%         else
%             % backward
%             motionDir(timeIdx) = 3;
%         end
%     elseif LRcounts(1,timeIdx)>LRcounts(2,timeIdx)
%         % robot turning CW
%         motionDir(timeIdx) = 2;
%     else %LRcounts(1,i)<LRcounts(2,i)
%         % robot turning CCW
%         motionDir(timeIdx) = 4;
%     end
    
    %% For each time determine new (error free) location
    % qt: odometry data 'u bar'
    
    qt(:,timeIdx+1) = odomupdate(qt(:,timeIdx),wheelDistances(:,timeIdx),WheelToWheelAvg);
    
    
    % sample motion model odometry
    %alpha = [1,1,1,1];
    %statecluster{timeIdx+1} = sample_motion_model_odometry(qt(:,timeIdx:timeIdx+1));
    
    
    % cumulative counts sp far
    %CumsumCounts = cumsum(counts,2);

end

% %% odometer plot
% hodom = figure;
% subplot(2,1,1)
% plot(time,LRcounts(1,:),'-b',time,LRcounts(2,:),'-r')
% subplot(2,1,2)
% plot(time,wheelDistances(1,:),'.b')
% hold on
% plot(time,wheelDistances(2,:),'.r')


%% 
% convert counts to distance of each wheel
%wheelDistances = CumsumCounts*mmPerCount;

%% Trajectory plot
figure('Position',[520,100,560,420])
plot(qt(1,:),qt(2,:),'.b')
hold on
%% plot clusters
%
%

%% plot state from encoder: x y th
% figure('Position',[620,100,560,420])
% subplot(3,1,1)
% plot(qt(1,:))
% subplot(3,1,2)
% plot(qt(2,:))
% subplot(3,1,3)
% plot(qt(3,:))
%keyboard
