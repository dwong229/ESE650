function [robotState,map] = mySLAM(edata,imudata,hdata,kinectModel)

global robotState map

% Encoder ahd IMU only so far

%% Run slam given ecoder data, imu data and lidar data as well as kinect data if
% available.

% INPUT
% edata: Encoder data: edata.ts = time[1 x n], edata.counts = [4 x n]
% imudata: IMU data: imudata.ts = time[1xm], imudata.vals = [6xm]
% OUTPUT
% robotstate: robotstate.vals = [6xp], robotstate.ts = [6xp]

%% User options
% set plot option
liveplot = false;
mapplot = false;
slamplot = true;
startplottime = 6; % when to start visualizing robot

% sensor modes
runIMU = true;
runOdom = true;
runLidar = true;
runKinect = false;

% options for particle filter
particlefilter = true;
numparticles = 10;
particles = struct('state',zeros(6,numparticles),'probability',ones(1,numparticles)/numparticles,'weight',ones(6,numparticles));
    
%% calibrate times:
startTime = min([edata.ts(1),imudata.ts(1),hdata.Hokuyo0.ts(1)]);

% encoder
tE = edata.ts - startTime;
encoder = edata.counts;

% IMU
tI = imudata.ts - startTime;
[Ax,Ay,Az,Wx,Wy,Wz] = IMUfile2BodyFrame(imudata.vals);
IMUData = [Ax;Ay;Az;Wx;Wy;Wz];

% Lidar
if ~runLidar
    tL = inf;
else
    tL = hdata.Hokuyo0.ts - startTime;
    if mapplot
        h_mapfig = figure;
        h_mapdata = imagesc(zeros(2));
    end
end

% kinect
%tK = startTime - startTime;
tK = inf;

sensorTime = {tI,tE,tL,tK}


% initialize plot
if liveplot
    h_xy = figure;
    h_trajhistory = plot(0,0,'.b');
    [h_robotnow,h_robotcenter] = robotplot(zeros(3,1));
    timeStr = 'time: 0 sec';
    hTitle = title(timeStr);
    %axis([-1e4 30e3 -1e4 30e4])
    axis equal
end

if slamplot
    h_slamfig = figure;
    timeStr = 'time: 0 sec';
    h_slamtitle = title(timeStr);
    hold on
    h_slammap = imagesc(zeros(2));
    h_slamtraj = plot(0,0,'.k');
    colormap(gray)
    axis equal
    
end

% initialize time and indices
stateIdx = 1;
% keep track of which idx was last used for updating state
currentIdx = zeros(1,4); %[imu Odom lidar kinect]
% determine which
timeMaxIdx = [length(tI) length(tE) length(tL) length(tK)];
timeNext = [tI(1) tE(1) tL(1) tK(1)];

% initialize robotstate
robotState = struct('state',zeros(6,sum(timeMaxIdx)),'ts',zeros(1,sum(timeMaxIdx)),'map','');

% initialize map
mapres = [100,100]'; % 100mm = .1m per grid
map.xmin = -10e3;
map.xmax = 60e3;
map.ymin = -10e3;
map.ymax = 60e3;
map.sizex = ceil((map.xmax - map.xmin) / mapres(1) + 1); %cells
map.sizey  = ceil((map.ymax - map.ymin) / mapres(2) + 1);
map.gridmap = zeros(map.sizex,map.sizey,'int8');
map.resolution = mapres;
map.offset(1,1) = find([map.xmin:map.resolution(1):map.xmax] == 0);
map.offset(2,1) = find([map.ymin:map.resolution(2):map.ymax] == 0);

timeEnd = false;
timeEndIdx = sum(timeMaxIdx.*[runIMU,runOdom,runLidar,runKinect]);
t0 = CTimeleft(timeEndIdx);
%while ~timeEnd
for i = 1:timeEndIdx
    t0.timeleft();
    %while stateIdx<1000
    stateIdx = stateIdx+1;
    
    % determine which sensor update is next
    [~,nextSensorUpdate] = min(timeNext);
    
    % update currentIdx for nextSensor
    currentIdx(nextSensorUpdate) = currentIdx(nextSensorUpdate) + 1;
    
    % update robotState.time
    robotState.ts(stateIdx) = timeNext(nextSensorUpdate);
    
    if nextSensorUpdate == 1
        %% IMU
        %fprintf('Itime: %7.4f \n',timeNext(nextSensorUpdate))
        % execute state update - motion update
        %stateIMU = imuUpdate(IMUData(:,startIdx:endIdx),tI(startIdx:endIdx));
        % use IMU to update pitch/roll/yaw only state(5)
        if currentIdx(1) == 1
            dt = [0 tI(1)];
            robotState.state(:,stateIdx) = imuUpdate(IMUData(:,currentIdx(1)),dt,zeros(6,1));
        else
            dt = [tI(currentIdx(1)-1) tI(currentIdx(1))];
            robotState.state(:,stateIdx) = imuUpdate(IMUData(:,currentIdx(1)),dt,robotState.state(:,stateIdx-1));
        end
        
        %         %% Troubleshoot - turn off UKF:
        %         if stateIdx == 1
        %             robotState.state(:,stateIdx) = zeros(6,1);
        %         else
        %             robotState.state(:,stateIdx) = robotState.state(:,stateIdx-1);
        %
        %         end
        
    elseif nextSensorUpdate == 2
        %% encoder
        %numtime=numel('Etime: %7.4f');
        %fprintf('\b\b\b\b\b\b\b');
        %fprintf('Etime: %7.4f \n',timeNext(nextSensorUpdate))
        % execute state update - motion update
        % use encoder to update x,y,z position
        if currentIdx(2) == 1
            dt = tE(1);
            if stateIdx == 1
                robotState.state(:,stateIdx) = encoderUpdate(encoder(:,currentIdx(2)),dt,zeros(6,1),kinectModel);
            else
                robotState.state(:,stateIdx) = encoderUpdate(encoder(:,currentIdx(2)),dt,robotState.state(:,stateIdx-1),kinectModel);
            end
        else
            dt = diff(tE(currentIdx(2)-1:currentIdx(2)));
            robotState.state(:,stateIdx) = encoderUpdate(encoder(:,currentIdx(2)),dt,robotState.state(:,stateIdx-1),kinectModel);
        end
    elseif nextSensorUpdate == 3
        %% lidar
        % Measurement update!
        % robot state stays the same
        %disp('Map Update')
        % update map
        %fprintf('\b\b\b\b\b\b\b');
        %fprintf('Ltime: %7.4f \n',timeNext(nextSensorUpdate))
        
        if currentIdx(3) == 1
            % update map
            map = lidarUpdate(robotState.state(:,stateIdx-1),hdata,currentIdx(3),map);
            robotState.state(:,stateIdx) = robotState.state(:,stateIdx-1);
        elseif particlefilter
            % generate particles to pick best state           
            % Update state
            robotState.state(:,stateIdx) = lidarParticleUpdate(robotState.state(:,stateIdx-1),hdata,currentIdx(3),map);
            % update map
            map = lidarUpdate(robotState.state(:,stateIdx),hdata,currentIdx(3),map);
        else
            map = lidarUpdate(robotState.state(:,stateIdx-1),hdata,currentIdx(3),map);
            robotState.state(:,stateIdx) = robotState.state(:,stateIdx-1);
        end
        
        % plot map
        if mapplot
            figure(h_mapfig);
            %mapLimits = size(map.gridmap).*map.resolution';
            mapLimits = size(map.gridmap);
            if currentIdx(3) == 1
                h_mapdata = imagesc(flipud(map.gridmap));
            elseif startplottime < robotState.ts(stateIdx)
                set(h_mapdata,'XData',[1 mapLimits(2)],'YData',[1 mapLimits(1)],'CData',flipud(map.gridmap))
            end
            drawnow
        end
        

    else
        %kinect
        
    end
    
    %% Live trajectory plot
    if liveplot %&& startplottime < robotState.ts(stateIdx)
        % PLOT traj
        figure(h_xy)
        set(h_trajhistory,'XData',robotState.state(1,1:stateIdx),'YData',robotState.state(2,1:stateIdx));
        robotplot(robotState.state([1,2,6],stateIdx),h_robotnow,h_robotcenter);
        timeStr = sprintf('time: %7.4f sec',timeNext(nextSensorUpdate));
        set(hTitle, 'String',timeStr);        
    end
    
    %% SLAM Plot
    if slamplot && mod(stateIdx,500) == 0 && currentIdx(3)>1
        % update map
        figure(h_slamfig)
        
        mapLimits = size(map.gridmap);
        
        set(h_slammap,'XData',[1 mapLimits(2)],'YData',[1 mapLimits(1)],'CData',map.gridmap)
        
        % update traj
        % convert to grid coords:
        offset = map.offset + [1;1]; %[rows,col]
        xy = robotState.state(1:2,:);
        xy = floor(bsxfun(@rdivide,robotState.state(1:2,:),map.resolution));
        % adjust traj coords to map coords based on offset
        gridtrajx = xy(1,:) + offset(2);
        %gridtrajy = size(map.gridmap,1) - offset(1) - xy(2,:);
        gridtrajy = xy(2,:) + offset(1);
        
        set(h_slamtraj,'XData',gridtrajy,'YData',gridtrajx)
        
        timeStr = sprintf('time: %7.4f sec',timeNext(nextSensorUpdate));
        set(h_slamtitle, 'String',timeStr);
    end
    
    %% Particle Filter
    % generate particles
    %particle.state = genParticles(robotState.state(:,stateIdx),diff(robotState.state(:,stateIdx-1:stateIdx),1,2),numparticles);
    
    % 
    
    
    %% update timeVectors
    if timeMaxIdx(nextSensorUpdate) == currentIdx(nextSensorUpdate)
        % reached end of times
        timeNext(nextSensorUpdate) = inf;
    else
        % update timenext with
        timeNext(nextSensorUpdate) = sensorTime{nextSensorUpdate}(currentIdx(nextSensorUpdate)+1);
    end
    if all(isinf(timeNext))
        timeEnd = true;
        break
    end
    
    
end % while loop: state update
disp('All data analysed')
% plot current state and past trajectory
if ~liveplot
    figure
    plot(robotState.state(1,:),robotState.state(2,:),'.b')
    title('Trajectory')
end
