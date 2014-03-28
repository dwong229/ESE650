function [] = checwfromVicon(RVicon,tV,angvelVicon)

% Sanity check: Numberically integrates angular velocity from vicon to check if it
% matches R from vicon
% INPUT
% - tV: time vector
% - RVicon: Rotation matrix from vicon
% - angvel: from vicon

% Output
% - plots a figure

Rlast = RVicon(:,:,1);
dtVicon = diff(tV);

figure
for i = 1:length(dtVicon)
    
    subplot(1,2,1)
    rotplot(RVicon(:,:,i+1),tV(i+1));
    
    subplot(1,2,2)
    Rnow = rotx(angvelVicon(1,i)*dtVicon(i))*roty(angvelVicon(2,i)*dtVicon(i))*rotz(angvelVicon(3,i)*dtVicon(i))*Rlast;
    rotplot(Rnow,tV(i+1));
    Rlast = Rnow;
end

disp('Function: checwfromVicon. Close figure when done. Then fit F5')
keyboard