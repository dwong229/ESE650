function [] = checwfromViconGyro(tV,angvelVicon,tI,Wx,Wy,Wz)

figure
subplot(2,1,1)
plot(tV(1:end-1),angvelVicon(1,:),'.r','MarkerSize',5)
hold on
plot(tV(1:end-1),angvelVicon(2,:),'.g','MarkerSize',5)
plot(tV(1:end-1),angvelVicon(3,:),'.b','MarkerSize',5)
xlabel('Time')
ylabel('rad/sec')
legend('X','Y','Z')
title('Vicon Ang Vel in World Frame')

subplot(2,1,2)
plot(tI,Wx,'.r')
hold on
plot(tI,Wy,'.g')
plot(tI,Wz,'.b')
xlabel('Time')
ylabel('arb units')
title('Gyro in Body Frame')
legend('X','Y','Z')
hold off

disp('Function: checwfromViconGyro. Close figure when done. Then fit F5')
keyboard