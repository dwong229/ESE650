function coord = PositionRangeToCoord(state,angle,range)

% Takes lidar data with position of lidar and returns coordinates of range
% sensor readings
%% INPUT
% state = [x y z r p y], world coordinate of robot
% angle = angle in radians of range sensor 
% range = lidar data
%% OUTPUT
% coordinates in world frames [x;y] (mm)

% initialize body coords of range measurements
bodyCoord = zeros(4,length(angle));
bodyCoord(4,:) = 1; % homogeneous coord
%bodyCoord(1:2,:) = repmat(range',[2,1]).*[sin(-angle');cos(angle')];
bodyCoord(1,:) = range.*sin(-angle);
bodyCoord(2,:) = range'.*cos(angle');

%% transform body coord to world coord
% translate robot:
%sensor to body transform for imu
%Tsensor = trans([0 301.83/2 545.44])*rotz(0)*roty(0)*rotx(0);
%Tsensor = trans([.1 0 0])*rotz(0)*roty(0)*rotx(0);
Tsensor = [1 0 0 .1;0 1 0 0;0 0 1 0;0 0 0 1];
%transform for the imu reading (assuming zero for this example)
%Timu = rotz(state(6))*roty(state(5))*rotx(state(4));
Timu = rotzyx(state([6,5,4]));

%body to world transform (initially, one can assume it's zero)
Tpose   = trans(state(1:3));

T = Tpose*Timu*Tsensor;
coord = T*bodyCoord;
%T = trans(state(1:3));
% rotation 
%R = rotxyz(state(4),state(5),state(6));
%R = rotz(state(6));
%coord = T*R*bodyCoord;


%% Troubleshooting plot
% h1 = figure;
% plot(bodyCoord(1,:),bodyCoord(2,:),'.b')
% hold on
% plot(coord(1,:),coord(2,:),'.r')
% 
% axis equal
% keyboard
% close(h1)
