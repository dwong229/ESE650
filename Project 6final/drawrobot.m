function robotposn = drawrobot(state,varargin)

% Given the state of the robot, draw a C shaped robot.
% if a figure is passed in, add robot to that figure:
% state in [i,j,th]

%% Unpack states
IJtrans = zeros(2,1);
IJtrans(:) = state(1:2);
th = state(3);


%% raw coodinates of the robot in C configuration
IJbody = flipud([0 10;20 10;20 30;-20 30;-20 -30;20 -30;20 -10;0 -10]'); %[i;j]
beadRad = 15;
%d = sqrt(beadRad^2 - 10^2) + 20;
% d = 10;
% IJbody(2,:) = IJbody(2,:);

IJworld = [cos(th) sin(th);-sin(th) cos(th)] * IJbody;
IJworld = bsxfun(@plus,IJworld,IJtrans);

%% output robotposn
robotposn = IJworld; %[i;j] = [y;x]

%figure
%subplot(1,2,1)
%fill(IJbody(2,:),IJbody(1,:),'b')

%subplot(1,2,2)
%fill(IJworld(2,:),IJworld(1,:),'r')
%title(num2str(state))