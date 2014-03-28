clear all
close all
clc
%% Project 3 gesture recognition
% load training data file
load('trainingdata.mat')
load('traincropped.mat')

numTrainingData = length(train);

%% Use cross validation
% generate number from length of train without replacement
FileRandomIdx = randperm(length(train));

% size of training set (out of 30 trajectories)
TrainingSetSize = 30;

% number of different states to deduce
NumOfStates = 2^3;
AllTrainingSets = []; % initialize matrix to hold all data
TrainDataWorld = cell(TrainingSetSize,1);

%% plot options

%% Files with huge time discrepencies
%FileRandomIdx(6) = 31;
%FileRandomIdx(11) = 32;
%FileRandomIdx(19) = 33;
%FileRandomIdx(26) = 34;
useworldData = true;

if ~useworldData
    % use generated worldData with UKF already run.
    worldData = traincroppedToWorldData(traincropped);
    
    
else
    disp('Using precomputed data')
    load('worldData.mat')
end

% put trainingdata into a matrix
TestSetIdx = 0;
TrainingSetIdx = 0;
TrainingSet(1).labelsIdx = worldData(1).labelsIdx;
for i = 1:30
    if FileRandomIdx(i)<=TrainingSetSize
        % training data
        TrainingSetIdx = TrainingSetIdx + 1;
        %% Put all data in a big matrix
        % [wx wy wz ax ay az]' - world frame [ n x 6]
        AllTrainingSets = [AllTrainingSets worldData(i).data]; % 6xn n is number of data
        TrainingSet(TrainingSetIdx).data = worldData(i).data;
        TrainingSet(TrainingSetIdx).label = worldData(i).label;
        TrainingSet(TrainingSetIdx).filename = worldData(i).filename;
    else
        TestSetIdx = TestSetIdx + 1;
        TestSet(TestSetIdx).data = worldData(i).data;
        TestSet(TestSetIdx).label = worldData(i).label;
        TestSet(TestSetIdx).filename = worldData(i).filename;
    end
end

clear i TrainingSetIdx TestSetIdx
keyboard
%% Train states by making NumOfStates clusters of gaussians
[mu A Ps] = LearnStates(AllTrainingSets,NumOfStates);
emissionModel = struct('mu',mu,'A',A,'Ps',Ps);
TrainIdx = 0;
TestIdx = 0;
%% For each dataset in the training set, determine states at each time:
for i = 1:30
    
    %% Train data!
    %hcircle = figure;
    %hfig8 = figure;
    %hfish = figure;
    %hhammer = figure;
    %hpend = figure;
    %hwave = figure;
    
    
    disp(i)
    %% Display filename
    strfile = strcat('DataType: ',train(1).labelsIdx(train(i).label),' File: ',train(i).filename);
    disp(strfile)
    
    IMUData = worldData(i).data;
    tI = worldData(i).data(1,:)';
    %% determine which state each observation is from:
    TrainingSetState = DetermineObservationState(IMUData,mu,A);
    
    %plot(TrainingSetState)
    %axis([1 length(tI) 1 NumOfStates])
    %title(strfile)
    %keyboard
    if FileRandomIdx(i)<=TrainingSetSize
        TrainIdx = TrainIdx + 1;
        % save trainingsetstate
        TrainingSet(TrainIdx).state = TrainingSetState;
    else
        TestIdx = TestIdx + 1;
        TestSet(TestIdx).state = TrainingSetState;
    end
end

%% Train HMM given trainingset observations and states

% Train a different model for each gesture
[modelA, modelB] = runHMMtrain(TrainingSet,mu,A,Ps);

save('HMMModel.mat','modelA','modelB','emissionModel')

%%
disp('Training Verification')
nTrain = length(TrainingSet);
correctCount = zeros(1,nTrain);
for testIdx = 1:nTrain
    % Use HMM to classify trajectory
    testseq = TrainingSet(testIdx).state';
    trueseqclassification = TrainingSet(testIdx).label;
    
    [sortedP, classification] = classifyHMM(testseq,modelA,modelB,'dw');
    
    if classification(1) == trueseqclassification;
        correctCount(testIdx) = 1;
    else
        disp('incorrect classification')
        classification
        trueseqclassification
        keyboard
    end
end

classificationsuccess = sum(correctCount)/nTrain

%%
disp('------------------')
disp('Cross Validation Verification')
nTest = length(TestSet);
correctCount = zeros(1,nTest);
for testIdx = 1:nTest
    % Use HMM to classify trajectory
    testseq = TestSet(testIdx).state';
    trueseqclassification = TestSet(testIdx).label;
    
    [prob, classification] = classifyHMM(testseq,modelA,modelB,'dw');
    
    if classification(1) == trueseqclassification;
        
        correctCount(testIdx) = 1;
        keyboard
    else
        disp('incorrect classification')
        classification
        trueseqclassification
        keyboard
    end
end
classificationsuccess = sum(correctCount)/nTest

