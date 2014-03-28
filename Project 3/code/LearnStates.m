function [ctrs A Ps] = LearnStates(AllTrainingSets,NumOfStates)

% given training data 6features x n observations cluster into NumOfStates
% return mean mu, covariance matrix and probabiliy of each state

AllTrainingSets = AllTrainingSets'; % rotate so that each row is one observation
[idx, ctrs] = kmeans(AllTrainingSets,NumOfStates);

% initialize covariance matrix
A = cell(NumOfStates,1);
N = hist(idx,[1:NumOfStates]);
for i = 1:NumOfStates
    stateIdx = 1:length(idx); % initiate stateIdx
    stateIdx(idx ~= i) = [];
    A{i} = inv(cov(AllTrainingSets(stateIdx,:))); % inverse covariance matrix
   %keyboard
end

% compute probability of each color
Ps = N/length(idx);
