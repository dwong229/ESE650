function [newState] = imuUpdate(imudata,dt,lastState)

% Update statevector based on IMU data.  Update orientation only onto posn
% INPUT
% imudata: 
%
%
% OUTPUT
%
%
%

if dt(1) == 0
    % initiating UKF
    stateIMU = UKF(imudata,dt(2),'first');
else
    stateIMU = UKF(imudata,diff(dt));
end

rpy = tr2rpy(QuatToRot(stateIMU(1:4)));

if any(isnan(rpy))
    disp('imuUpdate: rpy is nan')
    %keyboard
    rpy = lastState(4:6);
end

newState = lastState;
newState(4:6) = rpy;