function particlestate = genParticles(state,dstate,n)

% generate n-particles from state using normal distribution
% perterb x,y,yaw only
% INPUT
% state: either [6x1] or [6xn]
% n: number of particles to generate
%
% OUPUT
% particlestate: [6xn] xyzrpy state matrix for n particles


% set perturbation distribution (normal)
mu = dstate([1,2,6],1);
sigma = [250 250 deg2rad(20)]; %mm

% generate 10 particles
dp = mvnrnd(mu,sigma,n);

% add to state
if size(state,2) == 1
    particlestate = repmat(state,[1,n]);
    particlexyth = bsxfun(@plus,dp',state([1 2 6],1));
elseif size(state,2) == n
    particlestate = state;
    particlexyth = state + dp';
else
    disp('genParticles: invalid state input size.')
    keyboard
end

particlestate(1,:) = particlexyth(1,:);
particlestate(2,:) = particlexyth(2,:);
particlestate(6,:) = particlexyth(3,:);


%figure
% plot
%for i = 1:n
%    robotplot(particlestate(:,i));
%end

%hold on
%plot(state(1),state(2),'xr')
%xlabel('X (mm)')
%ylabel('Y (mm)')