%% Extract mp4
path = pwd;
mov = vid2img('Place12crop.mp4',path,'gray');

img = mov(1).imgdata;

hfig = figure;
h(1) = imshow(img,[1 255]);
%http://www.mathworks.com/help/images/examples/detect-and-measure-circular-objects-in-an-image.html
%d = imdistline;
circledia = [14 30];
tic
[centers, radii] = imfindcircles(img,circledia,'ObjectPolarity','dark','Sensitivity',0.8);
toc
% sensitivity of .8 - does not find robot or oblong bead clusters.
% (increasing sensitivity = more circle)

h(2) = viscircles(centers,radii);

% bead radii for settled bead:
beadradii = 16;
buffer = 3;
% determine which beads are candidate for moving.
bead.centers = centers(radii<beadradii+buffer,:);
bead.radii = radii(radii<beadradii+buffer);

% visualize beads that are candidate for moving in blue
h(3) = viscircles(bead.centers,bead.radii,'EdgeColor','b');

%% Find all other blobs
% These are obstacles

% Threshold image
thresh = 0.8;
bw = im2bw(img,thresh);

%figure;
%imshow(bw)

% identify blobs
% flip bw so that light regions (1) are detected
cc = bwconncomp(~bw);
regionStat = regionprops(cc,'Area','MajorAxisLength','MinorAxisLength','Centroid','PixelIdxList');
   
% determine which are not already identified as circles
regionCentroid = cat(1,regionStat.Centroid);

obstacleIdx = logical(ones(1,length(regionStat)));

% plot all regions
%plot(regionCentroid(:,1),regionCentroid(:,2),'xb')

for i = 1:length(obstacleIdx);
    d = dist([regionCentroid(i,:)' centers']);
    dmin = min(d(2:end,1));
    if dmin < 5
        % region is a circle
        obstacleIdx(i) = false;
    end
end


% Obstacles:
% plot
figure(hfig)
%h(3) = plot(regionCentroid(obstacleIdx,1),regionCentroid(obstacleIdx,2),'xb')
obstaclePix = cat(1,regionStat(obstacleIdx).PixelIdxList);
%img(obstaclePix) = 0;
h(3) = viscircles(regionCentroid(obstacleIdx,:),cat(1,regionStat(obstacleIdx).MajorAxisLength),'EdgeColor','y');


%set(h(1),'CData',img)


% Plot image with which spheres are out of plane, moveable and which are obstacles


%% Model for bead jiggle

%% Model for bead falling

%% 