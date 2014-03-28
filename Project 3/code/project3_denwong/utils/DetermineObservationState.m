function [TrainingSetState] = DetermineObservationState(O,mu,A)
    
%% determine which state each observation is from:

% INPUTS
% O: 6xn observations
% model: mu, A, Ps
% mu: mean of each state
% A: cell of covariance matrices
% Ps: probability of each state

nO = size(O,2); % number of observations
nS = length(A); % number of states
probOfState = zeros(nO,nS);

% For each observation compute the probability of each state
for s = 1:nS
    probOfState(:,s) = mvnpdf(O',mu(s,:),inv(A{s}));
end

% classify state
[~,TrainingSetState] = max(probOfState,[],2);
