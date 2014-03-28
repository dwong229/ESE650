function [map,WorldCoordGrid] = checkOffset(WorldCoordGridRaw,map)

%% check to see if WorldCoordGrid Raw are all positive if not shift offset
% and shift new points and existing map.
% INPUTS
% WorldCoordGridRaw : [2 x n] new rows and colums points from lidar for map
% map : struct .gridmap: occupancy grid, .resolution: [x y] mm per grid
% .offset: where ref robot position starts [row,col]
% OUPUTS
% map
% WorldCoordGrid
%

oldoffset = map.offset;
newoffset = [0 0]';

minGrid = min(WorldCoordGridRaw,[],2);

% check x coordinate
if minGrid(1) < 1
    newoffset(1) = -minGrid(1) + oldoffset(1);
end

% check y coordinate
if minGrid(2) < 1
    newoffset(2) = -minGrid(2) + oldoffset(2);
end

if any(minGrid<1)
    WorldCoordGrid = bsxfun(@plus,WorldCoordGridRaw,newoffset);
    % add pad of zeros.
    
    mapTemp(newoffset(1)+1:newoffset(1)+size(map.gridmap,1),newoffset(2)+1:newoffset(2)+size(map.gridmap,2)) = map.gridmap;
    map.gridmap = mapTemp;
    map.offset = newoffset + oldoffset;
else
    WorldCoordGrid = WorldCoordGridRaw;
end


%% check if matrix is big enough at positive side
maxGrid = max(WorldCoordGrid,[],2);
if map.sizex < maxGrid(1)
    addrow = maxGrid(1)-map.sizex-1;
    % add appropriate # of rows
    map.gridmap(map.sizex+1:map.sizex+1+addrow,:) = 0;        
    map.sizex = size(map.gridmap,1);
end

if map.sizey < maxGrid(2)
    addcol = maxGrid(2)-map.sizey-1;
    % add appropriate # of rows
    map.gridmap(:,map.sizey+1:map.sizey+1+addcol) = 0;
    % update size:
    map.sizey = size(map.gridmap,2);
end



