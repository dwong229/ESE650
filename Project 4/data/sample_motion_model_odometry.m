function [xnew] = sample_motion_model_odometry(odom,stateLast);

%% Apply algorithm for sammpling from p(x_t | u_t,x_(t-1)) based on odometry 
% information.  Here the poast at time t : 

%persistent alpha1 alpha2 alpha3

xlast = odom(:,1);
xnow = odom(:,2);

%nsamples = 500;

% compute relative motion:
delta(1) = atan2(xnow(2) - xlast(2),xnow(1)-xlast(1)) - xlast(3);
delta(2) = sqrt((xnow(2) - xlast(2))^2 + (xnow(1)-xlast(1))^2);
delta(3) = xnow(3) - xlast(3) - delta(1);

%% generate random samples of alpha:
%robot-specific error parameters (specifies the error accrued with motion.)
aSample =[0 0 0 0];
%aSample = gensample();

%% compute delta hat
dhat(1) = delta(1) - (aSample(1)*delta(1)^2 + aSample(2)*delta(2)^2);
dhat(2) = delta(2) - (aSample(3)*delta(2)^2 + aSample(4)*(delta(1)^2 + delta(2)^2))
dhat(3) = delta(3) - (aSample(1)*delta(3)^2 + aSample(2)*delta(2)^2);

xnew = stateLast + [dhat(2)*cos(xlast(3) + dhat(1));dhat(2)*sin(xlast(3) + dhat(1));dhat(1) + dhat(3)];
