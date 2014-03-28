function [modelA modelB] = runHMMtrain(Obs,eMu,eA,ePs)

% function to run hmmtrain to build HMM for a given training set
% INPUT
% TrainingSet:
% eMu: emissions mean
% eA: emissions covariance
% ePs: emissions probability for each emission

% OUTPUT
% return model parameters optimized using Baum Welsh
%
%

% initialization
numClusters = 1+range(cat(1,Obs.state));
Obslabels = [Obs.label];

numEmissions = numClusters;
numStates = 4; % Train on this!!!!


%% initialize modelparameters
modelA = zeros(numStates,numStates,6);
modelB = zeros(numStates,numEmissions,6);

%% initialize HMM

for model = 1:6; % classification number from labelsidx
    
    modelIdx = Obslabels == model;
    modelIdx = find(modelIdx);
    
    % Figure out which states are most visited
    [n ~] = hist(cat(1,Obs(modelIdx).state),[1:numClusters]);
    
    % only consider state if visited more than 5%?
    %numEmissions = find(n>0);
    
    %% Initialize model parameters
    
    % transition matrix
    % Aij : probability of being in state i and going to state j
    alpha = 0.1;
    A = diag(ones(numStates-1,1)*alpha,1) + eye(numStates)*(1-alpha);
    A(end,1) = alpha;
    
    % marginal distribution
    pi = zeros(1,numStates);
    pi(1) = 1;
    % B: emission probabilities
    % these are defined by the clusters: clusterMu and clusterA
    %B = ones(numStates,numEmissions) * 1/numEmissions;
    if numEmissions == find(n>0)
        B = repmat(n(n>0)/sum(n),[numStates,1]);
    else
        B = repmat(n/sum(n),[numStates,1]);
        B = ones(numStates,numEmissions) * 1/numEmissions;
    end
    
    
    % package sequences of data
    seq = {Obs(modelIdx).state};
    % rotate sequences
    for seqIdx = 1:length(seq)
        seq{seqIdx} = seq{seqIdx}';
    end
    %keyboard
    [modelA(:,:,model),modelB(:,:,model)] = hmmtrain(seq,A,B,'Verbose',true);
    
end
%% Fix 0 probability error in B matrix:
disp('Add small probability to B matrix and renormalize')
modelBnew = modelB;
modelBnew(:,:,1) = modelB(:,:,1)+1e-10;

% normalize B:
normB = sum(modelBnew,2);
modelBnew = modelBnew./(repmat(normB,[1,8]));

modelB = modelBnew;
    
    
%     
%     for testIdx = 1:25
%         Obs(testIdx).label
%         strfile = strcat('DataType: ',Obs(1).labelsIdx(Obs(testIdx).label),' File: ',Obs(testIdx).filename);
%         disp(strfile)
%         %for segmentIdx = 1:4
%         
%         %% Given a model, return probabiliy that model was generated from this model
%         [pstates{testIdx},logpseq(testIdx)] = hmmdecode(Obs(testIdx).state',modelA,modelB);
%     end
%     plot(1:25,logpseq,'xb')
%     hold on
%     plot(modelIdx,logpseq(modelIdx),'or')
%     keyboard