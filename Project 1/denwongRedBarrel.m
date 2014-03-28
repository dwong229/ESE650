function [x,y,distance] = denwongRedBarrel(imgOrig)

    % load trained data
    load denwongTrain.mat
    
    plotOption = false;
    hsubfig = figure(2); 
    set(hsubfig,'Position',[67 130 797 600])
    
    % constants from training
    cr_scale = 1.75;
    
    % down sample image
    scaleTest = .75; % reduce size of img by this amt
    img = imresize(imgOrig, scaleTest);
    imgycbcr = double(rgb2ycbcr(img));
    imgycbcr(:,:,3) = imgycbcr(:,:,3)*cr_scale;
    
    %subplot(2,2,1)
    %imshow(img);
    
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
    subplot(2,2,1)
    imagesc(probabilityMat)
    title('Probability Map')
    axis image  
    
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
    

        greyMean = 0.008;
    
    BW = im2bw(probImg,greyMean);
    
    subplot(2,2,2)
    imshow(BW)
    title('Binarized Probability')

        
    % process BW:
    %BW = bwmorph(BW,'close');
    
    %BWfilt = bwmorph(BW,'dilate');
    BWfilt = bwmorph(BW,'erode');
    %BWfilt = bwmorph(BWfilt,'dilate');
    
    subplot(2,2,3)
    imshow(repmat(uint8(BWfilt),[1 1 3]).*img)
    title('Post Processed Binary Map Over Image')

    
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
    subplot(2,2,4)
    imshow(imgOrig)
    title('Barrel Identified')

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
    
    x = centroidOrig(1);
    y = centroidOrig(2);
    
    if plotOption
       image(img)
       hold on;
       plot(x,y,'g+')
       title(sprintf('Barrel distance %.lf m',distance));
        
    end
    
end