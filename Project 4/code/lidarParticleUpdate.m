function [beststate] = lidarParticleUpdate(laststate,hdata,idx,map)

% Given robot state,lidar measurement and current map, deduce best state using map_correlation
% and return best estimate for new state
% INPUTS
% state : [6x1]: [x y z r p y]
% hdata : 
% map: .gridmap: occupancy grid, .resolution: [x y] mm per grid 
% OUTPUT
% map

% unpack lidar data
ldata = hdata.Hokuyo0;
%pose = ldata.pose; % not needed
%res = ldata.res; % resolution
%tL = ldata.ts;
ranges = ldata.ranges; % ranges at each time (in m)
ranges = ldata.ranges*1e3; % convert to mm [each column is a dataset]
angles = ldata.angles; % angle in radians

numparticles = 200;
particles.state = genParticles(laststate,zeros(6,1),numparticles);
particles.state(:,numparticles+1) = laststate;

maxc = zeros(1,numparticles+1);
for parti = 1:length(maxc)
    guessstate = particles.state(:,parti);
    
    maxc(parti) = pickbeststate(guessstate,ranges(:,idx),angles,map);
    
    %proposedc = pickbeststate(guessstate,ranges(:,idx),angles,map);
    
end

[~,maxidx] = max(maxc);
beststate = particles.state(:,maxidx);
%if idx>300
%    keyboard
%end
%figure; plot3(particles.state(1,:),particles.state(2,:),maxc,'.b')
%hold on; plot(particles.state(1,maxidx),particles.state(2,maxidx),'og')

