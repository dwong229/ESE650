function [cmax] = pickbeststate(guessstate,ranges,angles,map)

% at each measurement update check to see which state is best
% update map

res = map.resolution(1); % mm/grid

%% Transformation of body coord to world coord:
lidarxy = PositionRangeToCoord(guessstate,angles,ranges);
%figure; plot(lidarxy(1,:),lidarxy(2,:),'.');

% limits of map
x_im = map.xmin:res:map.xmax; %x-positions of each pixel of the map
y_im = map.ymin:res:map.ymax; %y-positions of each pixel of the map

% limits of grid to search
n = 0;
x_range = -n*res:res:n*res;
y_range = -n*res:res:n*res;


% correlation
% x and y values are flipped! new world_x is also flipped to put into c:
c = map_correlation(map.gridmap*100, x_im, y_im, lidarxy(1:3,:), x_range,y_range);
%c = c/max(c(:));
%figure; surf(c);
%keyboard
% find best x and y
cmax = max(c(:));
%cmax = max(c(:));

%beststate = guessstate;
%beststate(1) = x_range(xidx);
%beststate(2) = y_range(yidx);
%beststate(1) = (x_range(xidx) - offset(2))*res(1);
%beststate(2) = (y_range(yidx) + offset(1) - size(map.gridmap,1))*res(1);

