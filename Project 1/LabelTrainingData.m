% ESE 650 Project 1
%
% Label training data

% OUTPUT [mu A Pcl] 

clear all
close all
clc
% Look for all .png files in folder
disp('Acquiring File Information...')
currentpath = genpath(cd);
[DirName]=uigetdir(currentpath,'Select Folder with Training Set Images');
addpath(genpath(DirName));

FileName = dir(strcat(DirName,'/*.png')); %linux
%FileName = dir(strcat(DirName,'\*.png')); %windows

% generate number from to length of FileName without replacement
FileNameRandomIdx = randperm(length(FileName));

% Size of training set (out of 50 images)
TrainingSetSize = 50;

% initialize dataset to store ROI
TrainingData = struct('filename','','dist','','ROIsize','','width','','height','','colormean','','colorstd','','centroid','');

%% Manually define ROI
ycbcrData = [];
if true
    % loop through each file and define a mask ROI around red bin
    % define ROI by mask of rows/columns of vertices of ROI.
    %for i = 1:length(FileName)
        for i = 1:10
        %if FileNameRandomIdx(i) < TrainingSetSize
        imgOrig = imread(FileName(i).name); %make sure FileName(i) is string
        
        %% 1. read text of filename up to period to determine distance of barrel
        distance = strtok(FileName(i).name,'.');
        
        %% 3. convert image into other colorspace
        imgycbcrOrig = rgb2ycbcr(imgOrig);
        
        %% 2. roughly box red barrel in a square
        disp('Box Barrel - does not have to be exact')
        BWOrig = roipoly(imgOrig);
        save(strcat(FileName(i).name(1:end-4),'_BW.mat'),'BWOrig')
        
        % look at pixels, at a distance bufferRegion, around BW to determine if color is very different
        % (use Euclidean between colors?)
        % down sample image
        scale = 1;
        img = imresize(imgOrig, scale);
        BW = uint8(imresize(BWOrig,scale));
        imgycbcr = rgb2ycbcr(img);
        
        % define a column vector that contains which pixels are in manually defined ROI
        BWidx = BW(:);
        
        % filter ycbcr channels
        ychannel = imgycbcr(:,:,1).*BW;
        cbchannel = imgycbcr(:,:,2).*BW;
        crchannel = imgycbcr(:,:,3).*BW;
        
        ycbcrDataTemp = [ychannel(crchannel~=0) , cbchannel(crchannel~=0),crchannel(crchannel~=0)];
                
        ycbcrData = [ycbcrData;ycbcrDataTemp];
        
        %% From smaller dataset, compute
        
    end
        % compute mean and std of red barrel in ycbcr space
               
        mu = mean(ycbcrData,1);
        
        % return A matrix

        A{1} = inv(cov(double(ycbcrData)));
        Pcl(1) = 1;
        save('ROIycbcrData','ycbcrData','mu','A','Pcl')
        
    end
%end

