function [] = visualizeSLAM(robotState,map)

%% Generate a new figure and plot trajectory and map

%% convert traj to grid map pts:

% convert to a grid of 1 grid = 100mm
% WorldCoordGridRaw : [x,y] -> [row,col]
gridtraj = mmToGrid(robotState.state,map.resolution,flipud(map.offset));

figure
imagesc(map.gridmap)
colormap(gray)
axis xy
axis equal
hold on
quiver(gridtraj(2,:),gridtraj(1,:),sin(robotState.state(6,:)),cos(robotState.state(6,:)),'r');