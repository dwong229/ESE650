function [sortedProb, classificationRank] = classifyHMM(seq,modelA,modelB,varargin)
    
%% Given a model, return probabiliy that model was generated from this model

if ~isempty(varargin)
    %disp('using dw b-f hmm algorithm')
    testdwcode = true
else
    disp('using matlab b-f hmm algorithm')
    testdwcode = false;
end



for modelIdx = 1:size(modelA,3)
    if testdwcode
       [logpseq(modelIdx),alpha,beta,scale] = dwHMMdecode(seq,modelA(:,:,modelIdx),modelB(:,:,modelIdx));
    else
        [~,logpseq(modelIdx),alpha,beta,scale] = hmmdecode(seq,modelA(:,:,modelIdx),modelB(:,:,modelIdx));
    end
end

logpseq(isnan(logpseq)) = -inf;

[sortedProb, classificationRank] = sort(logpseq,'descend');
%keyboard