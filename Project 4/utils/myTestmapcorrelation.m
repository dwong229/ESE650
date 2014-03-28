% Test map_correlation


load('23datanew.mat')

hokuyoFile = 'Hokuyo23.mat';

load(hokuyoFile);
ldata = Hokuyo0;
% unpack
ranges = ldata.ranges; % ranges at each time (in m)
ranges = ldata.ranges*1e3; % convert to mm [each column is a dataset]
angles = ldata.angles; % angle in radians

idx = 1;

laststate = zeros(6,1);

%yaw_range = deg2rad(-20:5:20);
%yaw_range = 0;
%numparticles = 0;
%particles.state = genParticles(laststate,zeros(6,1),numparticles);
%particles.state(:,numparticles+1) = laststate;
particles.state = laststate;
cmax = [];
figure
for iyaw = 1:numparticles+1
    %guessstate = laststate;
    %guessstate(6) = laststate(6) + yaw_range(iyaw);
    guessstate = particles.state(:,iyaw);
    
    proposedc(iyaw) = pickbeststate(guessstate,ranges(:,idx),angles,map);
    
    robotplot(particles.state([1,2,6],iyaw));
    
%     
%     if isempty(cmax)
%         cmax = proposedc(iyaw);
%         bestangle = yaw_range(iyaw);
%     elseif cmax < proposedc(iyaw)
%         bestangle = yaw_range(iyaw);
%         cmax = proposedc(iyaw);
%     end
end

% plot(yaw_range,proposedc)
%xlabel('angle')
%ylabel('c')

figure
plot3(particles.state(1,:),particles.state(2,:),proposedc,'.b')
hold on 
plot3(0,0,proposedc(end),'or')
% find max
[~,maxidx] = max(proposedc);
plot3(particles.state(1,maxidx),particles.state(2,maxidx),proposedc(maxidx),'og')

% given this map, randomly generate particles and determine which is
% closest:




%c = map_correlation(map.gridmap,x_im,y_im,Y(1:3,:),x_range,y_range);

%% plot original lidar points
% figure(1);
% plot(xs1,ys1,'.')
% 
% %plot map
% figure(2);
% imagesc(map.gridmap); colormap(gray); axis xy
% 
% %plot correlation
% figure(3);
% surf(c)
% xlabel('x-axis')
% 
