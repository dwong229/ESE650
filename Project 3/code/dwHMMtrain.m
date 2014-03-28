function [Abest, Bbest] = dwHMMtrain(Obs,Ainit,Binit)


%% Use Baum Welsh optimization to determine Anew, Bnew such that P(Obs|Anew,Bnew)

% INPUTS
% Ob: {n} cells of sequences of emissions
% Ainit: [numStates x numStates] initial A matrix (cyclic HMM)
% Binit: [numStates x numEmissions] inital B matrix

% OUTPUTS
% Abest: [numStates x numStates]
% Bbest: [numStates x numEmissions]

% initialize 
Aold = Ainit;
Bold = Binit;
numStates = size(A,1);
numEmissions = size(B,2);
numSeq = length(Obs,1);

% initialize best model values
Abest = Aold;
Bbest = Bold;
LLbest = zeros(1,numSeq); % log likelihood from dwHMMdecode
for seqIdx = 1:numSeq
    [LLbest(seqIdx), normalpha, normbeta, scaling] = dwHMMdecode(seq,A,B)
    [~,LLbest(seqIdx),~] = dwHMMdecode(Obs{seqIdx},Abest,Bbest);
end

%% Compute zeta
% zeta(i,j,t) = P(Si,Sj,O|model)/P(O|model)

zeta


%% Compute gamma


%% Update model params
% Update A

% Update B

%% Check if new model params are better


%% 