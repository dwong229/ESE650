%% Script to Run HMM Data on new Test data
clear all
close all
clc

%% Extract data
disp('Choose folder that contains training data')
folder_name = uigetdir;

% extract raw data
rawdata = ExtractTestIMUData(folder_name);
% rawdata: .data, .filename

figure;
for i = 1:length(rawdata)
   rawdata(i).data = cropData(rawdata(i).data);
end
close all

% convert to worlddata
testdata = traincroppedToWorldData(rawdata,'test');

%% Extract Model
load('HMMModel.mat')
mu = emissionModel(1).mu;
A = {emissionModel.A};
label = {'circle','figure8','fish','hammer','pend','wave'};
%% Cycle through each dataset

for i = 1:length(testdata)
    disp('~~~~~~~')
    disp(testdata(i).filename)
    % 'worlddata' -> states
    testdata(i).state = DetermineObservationState(testdata(i).data,mu,A)';
    
    testseq = testdata(i).state;
    % classify using HMM model
    [sortedP,classification] = classifyHMM(testseq,modelA,modelB,'dw');
    
    % print probability of each model with percentage
    disp('Classification Rank')
    disp(strcat(label{classification(1)},':',num2str(sortedP(1))))
    disp('--')
    for strline = 2:6
        disp(strcat(label{classification(strline)},':',num2str(sortedP(strline))))
    end
    % validation if applicable
    keyboard
end
%%
% Write this output to a txt file!
