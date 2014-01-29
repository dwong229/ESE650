% ESE 650 Project 1

% Compute const for determining distance from camera:

% Hreal/distanceFromCam = Himage/focalLength
% constant = Himage*dist

% approx dist from cam = const/Himage

% Look for all .png files in folder
disp('Acquiring File Information...')
currentpath = genpath(cd);
[DirName]=uigetdir(currentpath,'Select Folder with Training Set Images');
%DirName = '/home/denise/Dropbox/ESE650/Project 1/train';
addpath(genpath(DirName));

FileName = dir(strcat(DirName,'/*.png')); %linux

%for i = 1:length(FileName)
for i = 1:50
    % show image
    disp(FileName(i).name);
    imgOrig = imread(FileName(i).name); %make sure FileName(i) is string
    imshow(imgOrig)
    
    % 1. read text of filename up to period to determine distance of barrel
    distance = str2num(strtok(FileName(i).name,'.'));
    
    %ginput for width
    disp('Click width of barrel')
    disp('Double click if ends of barrel are occluded')
    
    [xWidth yWidth] = ginput(2);
    hold on
    plot(xWidth,yWidth,'-g','LineWidth',4)
    WPixel = sqrt(diff(xWidth)^2 + diff(yWidth)^2);
    
    %ginput for height
    disp('Click Height of barrel')
    [xWidth yWidth] = ginput(2);
    plot(xWidth,yWidth,'-g','LineWidth',4)
    HPixel = sqrt(diff(xWidth)^2 + diff(yWidth)^2);
    
    % Compute Constant
    constW(i) = WPixel * distance
    constH(i) = HPixel * distance
    keyboard
end
