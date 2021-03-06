function bead = updatePushedBead(robotposn,bead,beadNum)

% given robotposn and which bead is being pushed, return new bead
% position
% INPUT
% 
% OUTPUT
% 
% 


beadPosn = bead.center(beadNum,:);
beadRad = bead.radii(beadNum,1);


% Extrapolate bead posn based on robotposn
th = robotposn(3);
d = sqrt(beadRad^2 - 10^2) + 20;
%d = beadRad;
d = 0;

bead.center(beadNum,:) = round(robotposn(1:2) + d*[sin(th),cos(th)]);

