function map = lidarUpdate(state,hdata,idx,map)

% Given robot state,lidar measurement and current map, update and return
% map
% INPUTS
% state : [6x1]: [x y z r p y]
% hdata : 
% map: .gridmap: occupancy grid, .resolution: [x y] mm per grid 
% OUTPUT
% map

% unpack lidar data
ldata = hdata.Hokuyo0;
%pose = ldata.pose; % not needed
%res = ldata.res; % resolution
%tL = ldata.ts;
ranges = ldata.ranges; % ranges at each time (in m)
ranges = ldata.ranges*1e3; % convert to mm [each column is a dataset]
angles = ldata.angles; % angle in radians

% extract coordinate of range data in mm in world frame
WorldCoordmm = PositionRangeToCoord(state,angles,ranges(:,idx));

% convert to a grid of 1 grid = 100mm
% WorldCoordGridRaw : [x,y] -> [row,col]
WorldCoordGridRaw = mmToGrid(WorldCoordmm,map.resolution,map.offset);

% check and shift offset if needed
[map,WorldCoordGrid] = checkOffset(WorldCoordGridRaw,map);

% Convert x,y to index
WorldCoordIdx = sub2ind(size(map.gridmap),WorldCoordGrid(1,:),WorldCoordGrid(2,:));

% upvote current map
map.gridmap(WorldCoordIdx) = map.gridmap(WorldCoordIdx) + 1;


% %% Adapted from Test Map Correlation
% sensor to body transform
% Tsensor = trans([0 301.83/2 545.44])*rotz(0)*roty(0)*rotx(0);
% 
% transform for the imu reading (assuming zero for this example)
% Timu = rotz(state(6))*roty(state(5))*rotx(state(4));
% 
% body to world transform (initially, one can assume it's zero)
% Tpose   = trans(state(1:3));
% 
% full transform from lidar frame to world frame
% T = Tpose*Timu*Tsensor;
% 
% xy position in the sensor frame
% xs0 = (ranges(:,1).*sin(-angles))';
% ys0 = (ranges(:,1).*cos(angles))';
% 
% convert to body frame using initial transformation
% X = [xs0;ys0;zeros(size(xs0)); ones(size(xs0))];
% Y=T*X;
% 
% transformed xs and ys
% xs1 = Y(1,:);
% ys1 = Y(2,:);
% 
% convert from meters to cells
% xis = ceil((xs1 - map.xmin) ./ map.resolution(1));
% yis = ceil((ys1 - map.ymin) ./ map.resolution(2));
% 
% check the indices and populate the map
% indGood = (xis > 1) & (yis > 1) & (xis < map.sizex) & (yis < map.sizey);
% inds = sub2ind(size(map.gridmap),xis(indGood),yis(indGood));
% 
% map.gridmap(inds) = map.gridmap(inds) + 1;

