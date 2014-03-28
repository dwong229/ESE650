function [handleLine,handleCenter] = robotplot(stateVecxyth,varargin)

% Given the stateVector of a robot, plot a triangle in that position:


robotscale = 1;

x = stateVecxyth(1);
y = stateVecxyth(2);
th = stateVecxyth(3);

% choose scale of robot
robotScale = 0.5;

% define original triangle
OriginalCorners = [0 2;1 -1;-1 -1;0 2]'*robotScale; %[2xn] = [x;y]
OriginalCorners(3,:) = 1;

% translate and rotate corners
T = [cos(th) -sin(th) x;sin(th) cos(th) y;0 0 1];
newCorners = T*OriginalCorners;


if ~isempty(varargin)
    handleLine = varargin{1};
    handleCenter = varargin{2};
    set(handleLine,'XData',newCorners(1,:),'YData',newCorners(2,:))
    set(handleCenter,'XData',x,'YData',y)
else
    handleLine = line(newCorners(1,:),newCorners(2,:));
    hold on
    handleCenter = plot(x,y,'.r');
    axis equal
end




