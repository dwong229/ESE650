function [th] = integratew(W,dt)

% input:
% W matrix, 3 x n, where each row is wx,wy,wz.  
% Assume body starts in identity position theta = [0 0 0];
%
% output:
% th matrix, 3 x n, where each row is thetax,thetay, thetaz
%  

%wx = W(1,:);
%wy = W(2,:);
%wz = W(3,:);

th = zeros(size(W));
wworld = zeros(size(W));
time = [0 cumsum(dt)];
R = zeros(3,3,size(W,2));
R(:,:,1) = eye(3);
for i = 1:length(dt)
    % not restricting 
    %R = rotxyz(th(1,i),th(2,i),th(3,i));
    %wworld(:,i) = R*W(:,i);
    %th(:,i+1) = th(:,i) + dt(i)*wworld(:,i);
    R(:,:,i+1) = dt(i)*skew(W(:,i))*R(:,:,i) + R(:,:,i);
    th(:,i+1) = tr2rpy(R(:,:,i+1));
end
keyboard
th(1,:) = mod(th(1,:),2*pi);
th(1,th(1,:)>pi) = th(1,th(1,:)>pi) - 2*pi;
th(2,:) = mod(th(2,:),2*pi);
th(2,th(2,:)>pi) = th(2,th(2,:)>pi) - 2*pi;
th(3,:) = mod(th(3,:),2*pi);
th(3,th(3,:)>pi) = th(3,th(3,:)>pi) - 2*pi;

if false
    figure
    %subplot(2,1,1)
    plot(time,th(1,:),'.r')
    hold on
    plot(time,th(2,:),'.b')
    plot(time,th(3,:),'.g')
    ylabel('\omega (rad)')
    legend('x','y','z')
    title('Integrated gyro data to RPY')
    
%     subplot(2,1,2)
%     plot(time,wworld(1,:),'.r')
%     hold on
%     plot(time,wworld(2,:),'.b')
%     plot(time,wworld(3,:),'.g')
%     xlabel('Time (s)')
%     ylabel('\omega (rad/s)')
%     legend('x','y','z')
end
    