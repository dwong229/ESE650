%Test dw HMM code
clear all 
close all
clc
load('HMMModel.mat')
load('worldData.mat')

mu = emissionModel.mu;
A = {emissionModel.A};


testseq = DetermineObservationState(worldData(28).data,mu,A)';

% classify using HMM model
[sortedP,classification] = classifyHMM(testseq,modelA,modelB)

[sortedP,classification] = classifyHMM(testseq,modelA,modelB,'dw')

% train HMM!

