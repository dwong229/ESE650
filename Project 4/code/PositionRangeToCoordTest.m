% Test script for PositionRangeToCoord

% test set: Translation
state = zeros(6,2);
state(2,2) = 1;
angle = [-atan2(1,10);0;atan2(1,10)];
range = [sqrt(101) 10 0;9/cos(atan2(1,10)) 9 0]';

% test set: Rotatioon
state = zeros(6,2);
state(6,2) = -atan2(1,10);
angle = [-atan2(1,10);0;atan2(1,10)];
range = [sqrt(101) 10 0;0 sqrt(101) 10]';


WorldCoords = PositionRangeToCoord(state(:,1),angle,range(:,1));

figure
[h1,h2] = robotplot(state([1 2 6],1));
hold on;
plot(WorldCoords(1,:),WorldCoords(2,:),'.b')

WorldCoords = PositionRangeToCoord(state(:,2),angle,range(:,2));
robotplot(state([1 2 6],2),h1,h2);
plot(WorldCoords(1,:),WorldCoords(2,:),'.r')
