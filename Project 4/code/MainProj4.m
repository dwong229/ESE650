%% Project 4
clear
%% Load images 

global robotState map


disp('Choose folder that contains training data')
%folder_name = uigetdir;

% for laptop
%folder_name = '/home/mrsl-think/Dropbox/ESE650/Project 4/data';
%folder_name = '/home/al/Dropbox/ESE650/Project 4/data';

% for desktop
folder_name = '/home/denise/Dropbox/ESE650/Project 4/data'; % For GottaGoFast


allfiles = dir(folder_name);

%input_file_num = input('Input file number (20,21,22,23,24):','s');
%file_num = str2num(input_file_num);
file_num = 23;

%% separate into different types of files:
[edata,imudata,hdata,kdata,kinectModel] = findfiles(allfiles,file_num);

%% Make a 2D map
%robotstate = genTrajectory(edata.Encoders,imudata);

%% SLAM
[robotState,map] = mySLAM(edata.Encoders,imudata,hdata,kinectModel);