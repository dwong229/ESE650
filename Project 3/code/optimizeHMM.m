function [A,B,mu] = optimizeHMM(Obs,clusterMu,clusterA,clusterPs)

% group all datasets that have the same label
% for model = 1:6
model = 1;

numState = 1+range(cat(1,Obs.state));

Obslabels = cat(Obs.label);

modelIdx = Obslabels == model;
modelIdx = find(modelIdx);

% Figure out which states are most visited

[n dist] = hist(cat(1,Obs(modelIdx).state),[1:numState]);

% only consider state if visited more than 5%?
numEmissions = find(n>0);


%% Initialize model parameters

% transition matrix
% Aij : probability of being in state i and going to state j
A = zeros(numState,numState);
% marginal distribution
pi = zeros(1,numState);
pi(1) = 1;
% B: emission probabilities
% these are defined by the clusters: clusterMu and clusterA
B = ones(; 

totalTransistions = sum(n) - length(modelIdx);
%% initialize transition matrix with observed states
for Ai = 1:length(numEmissions)
    for Aj = 1:length(numEmissions)
        for i = 1:length(modelIdx)
            % for each model update probabilities
            % compute number of same state transitions
            A(numEmissions(Ai),numEmissions(Aj)) = A(numEmissions(Ai),numEmissions(Aj)) + length(findstr(Obs(modelIdx(i)).state',[numEmissions(Ai) numEmissions(Aj)]));
        end
    end
end
% each row should sum to 1:
sumArow = sum(A,2);
sumArow(sumArow == 0) = 1;
A = bsxfun(@rdivide,A,sumArow);

%% 


for i = 1:length(modelIdx)
    Obsstate = Obs(modelIdx(i)).state;
    
    
    
    
    
    
end