function [newState] = encoderUpdate(encoder,dt,lastState,kinect)

%  Run encoder update for position based on last encoder reading for distance travelled
%  add translationg to lastState.
%
%% INPUT
%  encoder: raw encoder reading from one time step
%  dt: 
%  lastState: robot state [x;y;z;r;p;y]
%  kinect : true if 
%% OUTPUT
% 
%
%

%% Load appropriate parameters
if kinect == true
    %disp('Load specs with kinect')
    % encoder parameters FOR kinect robot!
    CountsPerRev = 360; % counts per revolution of the wheel/360deg
    WheelDiameter = 254; % mm
    %robot geometry:
    WheelAxis = 330.20; %mm (front wheel axis to back wheel axis)
    WheelToWheelInner = 311.15; % mm (inner left wheel to inner right wheel)
    WheelToWheelOuter = 476.25; % mm (outer left wheel to outer right wheel)
    WeffectiveRatio = 1.85;
else
    % encoder parameters FOR NO kinect robot!
    %disp('Load specs without kinect')
    CountsPerRev = 180; % counts per revolution of the wheel/360deg
    WheelDiameter = 165.1; % mm
    %robot geometry:
    WheelAxis = 330.20; %mm (front wheel axis to back wheel axis)
    WheelToWheelInner = 311.15; % mm (inner left wheel to inner right wheel)
    WheelToWheelOuter = 527.05; % mm (outer left wheel to outer right wheel)
    WeffectiveRatio = 1.85;
end

% computation
mmPerRev = pi*WheelDiameter;
mmPerCount = mmPerRev/CountsPerRev; 
WheelToWheelAvg = (WheelToWheelInner+WheelToWheelOuter)/2*WeffectiveRatio;

LRcounts = [mean(encoder([2,4],:),1);mean(encoder([1,3],:),1)]; % average counts for left and right wheels [2 x n]

% convert encoder counts to distance travelled
wheelDistances = LRcounts*mmPerCount;

q0 = lastState([1,2,6]);
if any(isnan(q0))
    disp('encoderUpdate: q0 NAN')
    keyboard
    q0(3) = 0;
end
%% compute translation in body frame
qt = odomupdate(q0,wheelDistances,WheelToWheelAvg);
%yaw = qt(3);
% correct for pitch: %replace qt(3) with z value
%qt = rotx(lastState(5),3)*[qt(1:2);lastState(3)];
qt = [qt(1:2);lastState(3)];

%newState = lastState;
%xyz = rotxyz(lastState(4),lastState(5),lastState(6))*
newState = lastState;
%newState(6) = yaw; %untested (encoder yaw update)
newState(1:3) = qt(1:3);

