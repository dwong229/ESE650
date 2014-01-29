% Main file for running find barrel
train = 2; % 1: kmean, 2: manual, 3: skip train

if train == 1
    clear all
    close all
    clc    
    train = 1;
    % Look for all .png files in folder
    disp('Acquiring File Information...')
    currentpath = genpath(cd);
    [DirName]=uigetdir(currentpath,'Select Folder with Training Set Images');
    %DirName = '/home/denise/Dropbox/ESE650/Project 1/train';
    addpath(genpath(DirName));
    
    FileName = dir(strcat(DirName,'/*.png')); %linux
    %FileName = dir(strcat(DirName,'\*.png')); %windows
    
    % generate number from to length of FileName without replacement
    FileNameRandomIdx = randperm(length(FileName));
    
    % Size of training set (out of 50 images)
    TrainingSetSize = 40;
    
    %keyboard
    % Train clusters and find mu and A:
    cr_scale = 2; % stretch cr color channel by cr_scale
    cr_scale = 1.75; % stretch cr color channel by cr_scale (to train on 50)
    k = 5; % number of clusters
    [mu A Pcl] = LearnColorsToLabel(FileName(FileNameRandomIdx<=TrainingSetSize),k,cr_scale);
    keyboard
    
elseif train == 2
    
    % FOR MANUALLY TRAINED DATA using LABELTRAININGDATA.m
    load ROIycbcrData.mat
    TrainingSetSize = 10; % for 10 files
    FileNameRandomIdx = 1:length(FileName);
    cr_scale = 1;
end

%% Compute probabilities for test set
disp('Compute probability of red in image')
TestSetIdx = 1:length(FileName);
TestSetIdx(FileNameRandomIdx<=TrainingSetSize) = [];
h_troubleshoot = figure;
h_final = figure;

% Find red pixels in
for i = 1:length(TestSetIdx)
    figure(h_troubleshoot)
    disp(FileName(TestSetIdx(i)).name)
    tic
    imgOrig = imread(FileName(TestSetIdx(i)).name);
    
    %% MY ALGORITHM starts here... returns [x,y,d]
    
    % down sample image
    scaleTest = .75; % reduce size of img by this amt
    img = imresize(imgOrig, scaleTest);
    imgycbcr = double(rgb2ycbcr(img));
    imgycbcr(:,:,3) = imgycbcr(:,:,3)*cr_scale;
    
    subplot(2,2,1)
    imshow(img);
    
    % compute probability for red
    imgycbcrVec = reshape(imgycbcr,length(img(:))/3,3);
    
    probabilityMat = zeros(size(imgycbcr,1),size(imgycbcr,2));
    
    l = 1;
    diffVec = bsxfun(@minus,imgycbcrVec,mu(l,:));
    
    %diffVecCell = mat2cell(diffVec,ones(length(diffVec),1),3);
    %probabilityVec = sqrt(det(A{l})/((2*pi)^3)) * exp(-1/2*diffVec*A{l}*diffVec')*Pcl(l);
    %probabilityVec = diag(probabilityVec);
    %probabilityVec = cellfun(@(x) sqrt(det(A{l})/((2*pi)^3)) * exp(-1/2*x*A{l}*x')*Pcl(l),diffVecCell);
    
    probabilityVec = mvnpdf(imgycbcrVec,mu(l,:),inv(A{l}));
    
    probabilityMat(:) = probabilityVec;
    
    % plot
    subplot(2,2,2)
    title('Probability Map')
    imagesc(probabilityMat)
    axis image
    
    fprintf('Time to compute probability map: %4.3f\n',toc)
    
    
    %% Threshold pixels to determine which are likely red
    % rescale probabilityMat
    probImg = (probabilityMat-min(probabilityMat(:)))/range(probabilityMat(:));
    BW = im2bw(probImg,graythresh(probImg));
    
    
    %[~,greyMean] = kmeans(probImg(:),2)
    %if min(greyMean)<greyMeanConst
    %    disp('Using default threshold')
    %    BW = im2bw(probImg,greyMeanConst);
    %else
    %    BW = im2bw(probImg,min(greyMean));
    %end
    
    if train == 2
        % for manual color
        greyMean = 0.1;
    else 
        greyMean = 0.008;
    end
    
    BW = im2bw(probImg,greyMean);
    
    subplot(2,2,3)
    title('Binarized Probability')
    imshow(BW)
        
    % process BW:
    %BW = bwmorph(BW,'close');
    
    %BWfilt = bwmorph(BW,'dilate');
    BWfilt = bwmorph(BW,'erode');
    %BWfilt = bwmorph(BWfilt,'dilate');
    
    title('Post Processed Binary Map')
    subplot(2,2,4)
    imshow(repmat(uint8(BWfilt),[1 1 3]).*img)
    
    %% Find connected regions
    cc = bwconncomp(BWfilt);
    regionStat = regionprops(cc,'BoundingBox','Area','Orientation','MajorAxisLength','MinorAxisLength','Solidity','Centroid','ConvexImage');
    
    % unpack stats
    area = cat(1,regionStat.Area);
    % only consider top 5 areas
    if length(area)>5
        [y,areai] = sort(area,'descend');
        idx = areai(1:5);
        area = area(idx);
    else
        idx = 1:length(area);
    end
    
    major = cat(1,regionStat(idx).MajorAxisLength);
    minor = cat(1,regionStat(idx).MinorAxisLength);
    ratio = major./minor;
    orientation = cat(1,regionStat(idx).Orientation);
    solidity = cat(1,regionStat(idx).Solidity);
    
    % determine which region is the barrel
    
    % Orientation should be +/- 90deg from xaxis
    orientPt = abs(90-abs(orientation))/90;
    
    % Major/Minor axis ratio 'from training data
    %ratioPt = ratio/
    
    % Solity Area/Convex Hull Area
    
    
    barrelPoints = (1-area/max(area)) + (1-solidity) + orientPt;
    
    [~,barrelIdx] = min(barrelPoints);
    
    centroid = regionStat(idx(barrelIdx)).Centroid;
    
    % Use shape statistics to find rectanglem determine coodinates of
    % center and width and height of rect
    
    % plot convex area
    
    
    % plot image and mark centroid, print distance from cam.
    figure(h_final);
    imshow(imgOrig)
    %centroidOrig = centroid/scaleTest;
    hold on
    %plot(centroidOrig(1),centroidOrig(2),'xb','MarkerSize',10);
    % plot major and minor axis:
    BoundingBox = (regionStat(idx(barrelIdx)).BoundingBox)/scaleTest;
    
    majorOrig = BoundingBox(4);
    minorOrig = BoundingBox(3);
    
    centroidOrig = [BoundingBox(1)+minorOrig/2,BoundingBox(2)+majorOrig/2];
    
    xminor = [centroidOrig(1) - minorOrig/2;centroidOrig(1) + minorOrig/2];
    ymajor = [centroidOrig(2) - majorOrig/2;centroidOrig(2) + majorOrig/2];
    plot([xminor],[centroidOrig(2);centroidOrig(2)],'-b','LineWidth',3)
    plot([centroidOrig(1);centroidOrig(1)],ymajor,'-b','LineWidth',3)
    
    
    % corners = [x y] coords
    corners = [BoundingBox(1:2);BoundingBox(1)+BoundingBox(3) BoundingBox(2);...
        BoundingBox(1), BoundingBox(2)+BoundingBox(4); BoundingBox(1)+BoundingBox(3), BoundingBox(2)+BoundingBox(4)];
    plot(corners(:,1),corners(:,2),'.g','MarkerSize',5)
    
    plot(BoundingBox(1)+minorOrig/2,BoundingBox(2)+majorOrig/2,'xg','MarkerSize',10);
    
    %% Compute distance of barrel
    disp('Computing Distance of Barrel')
    % if ratio is approximately 1.5 then whole barrel is likely found:
    load CameraCal.mat
    
    cW = mean(constW);
    cH = mean(constH);
    
    BarrelRatio = majorOrig/minorOrig;
    fprintf('BarrelRatio: %4.2f \n',BarrelRatio)   
    dist = [cW/minorOrig, cH/majorOrig];
    
    if abs(BarrelRatio - 1.5) < 0.2
        disp('Full Barrel is Mostly Visible/Detected')
        distance = mean(dist);
        
    else
        % if occluded, pick one dimension to use:
        disp('Barrel is Likely Occluded')
        if BarrelRatio > 1.8
            disp('Use barrel height')
            distance = dist(2);
        else
            disp('Use barrel width')
            distance = dist(1);
        end
    end
        disp('-----')
        actualdistance = str2num(strtok(FileName(TestSetIdx(i)).name,'.'));
        fprintf('Actual Distance: %4.0f \n',actualdistance)
        fprintf('Distance Approx: %4.1f \n',distance)
        disp('-----')
    
    
    keyboard
end

%     %% Predict distance
%     xtest = 100*ones(1,3);
%     for l = 1:5
%         p(l) = sqrt(det(A{l})/((2*pi)^3)) * exp(-1/2*(xtest - mu(l,:))*A{l}*(xtest - mu(l,:))');
%         q(l) = mvnpdf(xtest,mu(l,:),inv(A{l}))
%     end
%
