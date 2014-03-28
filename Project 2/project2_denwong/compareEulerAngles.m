function [orientationVicon,orientationIMU] = compareEulerAngles(tV,RVicon,tI,IMUrots)

% Compute Euler angles from Vicon:
    [orientationVicon] = rots2rpy(RVicon,'mat');
    
    % Compute UKF results Euler angles:
    [orientationIMU] = rots2rpy(IMUrots,'mat');
    
    %Plot comparison of Euler angles and Vicon
    figure
    subplot(3,1,1)
    plot(tV(tV>0),orientationVicon(1,tV>0),'.k','MarkerSize',2)
    hold on
    plot(tI,orientationIMU(1,:),'.r','MarkerSize',3)
    ylabel('X')
    subplot(3,1,2)
    plot(tV(tV>0),orientationVicon(2,tV>0),'.k','MarkerSize',2)
    hold on
    plot(tI,orientationIMU(2,:),'.g','MarkerSize',3)
    ylabel('Y')
    subplot(3,1,3)
    plot(tV(tV>0),orientationVicon(3,tV>0),'.k','MarkerSize',2)
    hold on
    plot(tI,orientationIMU(3,:),'.b','MarkerSize',3)
    ylabel('Z')
    xlabel('time (s)')