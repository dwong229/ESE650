% Visualize lidar data:

ldata = hdata.Hokuyo0;
pose = ldata.pose; % not needed
res = ldata.res; % resolution
ranges = ldata.ranges; % ranges at each time (in m)
ranges = ldata.ranges*1e3; % convert to mm [each column is a dataset]
angles = ldata.angles; % angle in radians
tL = ldata.ts;

bodyXYZ = [0, -301.83/2+298.33, 514.35+254/2];

% initialize map
%map = int8(zeros(4e4,4e4));

% extract coordinate of range data in mm in world frame
state = zeros(6,1);
WorldCoordmm = PositionRangeToCoord(state,angles,ranges(:,300));

% convert to a grid of 1 grid = 100mm
WorldCoordGrid = mmToGrid(WorldCoordmm,[1 1]',[0 0]');

figure
subplot(1,2,1)
plot(WorldCoordmm(1,:),WorldCoordmm(2,:),'.b')
axis equal
subplot(1,2,2)
plot(WorldCoordGrid(1,:),WorldCoordGrid(2,:),'.b')
axis equal