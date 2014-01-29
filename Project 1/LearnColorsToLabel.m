function [mu A Pcl] = LearnColorsToLabel(FileName,k,cr_scale)

% Take all images, downsample and determine which colors to learn

%
plot1 = false;
plot2 = true;

disp('LearnColorsToLabel...')

% generate number from to length of FileName without replacement
FileNameRandomIdx = randperm(length(FileName));

% initiate matrix to store colors [R G B], [Y CB CR]
AllColors = struct('RGB',[],'YCBCR',[]);

% loop through to determine
tic
for i = 1:length(FileName)
    %for i = 1:5
    imgOrig = imread(FileName(i).name); %make sure FileName(i) is string
    
    %% 3. convert image into other colorspace
    imgycbcrOrig = rgb2ycbcr(imgOrig);
    
    % down sample image
    scale = 0.1;
    img = imresize(imgOrig, scale);
    imgycbcr = rgb2ycbcr(img);
    
    % store colors
    RGB = zeros(length(img(:))/3,3);
    YCBCR = RGB;
    for i = 1:3
        temp1 = img(:,:,i);
        temp2 = imgycbcr(:,:,i);
        RGB(:,i) = temp1(:);
        YCBCR(:,i) = temp2(:);
        clear temp1 temp2
    end
    AllColors.RGB = [AllColors.RGB;RGB];
    AllColors.YCBCR = [AllColors.YCBCR;YCBCR];
    clear RGB YCBCR
end

if plot1
    disp('Troubleshoot: Plot Y Cr Cb')
    subplot(2,1,1)
    %hy =scatter(AllColors.YCBCR(:,1),AllColors.YCBCR(:,3),10,AllColors.RGB,'fill');
    hy =scatter(AllColors.YCBCR(:,1),AllColors.YCBCR(:,3));
    xlabel('Y')
    ylabel('Cr')
    subplot(2,1,2)
    %hb =scatter(AllColors.YCBCR(:,2),AllColors.YCBCR(:,3),10,AllColors.RGB,'fill');
    hb =scatter(AllColors.YCBCR(:,2),AllColors.YCBCR(:,3));
    xlabel('Cb')
    ylabel('Cr')
    
    figure
    hb =scatter3(AllColors.YCBCR(:,1),AllColors.YCBCR(:,2),AllColors.YCBCR(:,3));
    xlabel('Y')
    ylabel('Cb')
    zlabel('Cr')
end

%% Fit kmeans
XscaledData = AllColors.YCBCR;
% normalize data: (x - mean)/range
%rangemat = repmat(range(AllColors.YCBCR,1),[size(AllColors.YCBCR,1),1]);
%XscaledData = bsxfun(@minus,AllColors.YCBCR, mean(AllColors.YCBCR,1))./rangemat;

% maximize spread of Cr axis by 10% to increase weighting on Cr
XscaledData(:,3) = XscaledData(:,3)*cr_scale;

% kmeans
[idx, ctrs] = kmeans(XscaledData,k);
%ctrs = bsxfun(@plus,bsxfun(@times,ctrs,range(AllColors.YCBCR,1)), mean(AllColors.YCBCR));
%ctrs(:,3) = ctrs(:,3)/cr_scale;

X = AllColors.YCBCR;

%% plot clusters

if plot2
    figure
    plotstr = {'r.','b.','y.','g.','c.'};
    for i = 1:k
        plot3(X(idx==i,1),X(idx==i,2),XscaledData(idx==i,3),plotstr{i},'MarkerSize',2)
        hold on
    end
    plot3(ctrs(:,1),ctrs(:,2),ctrs(:,3),'kx')
    plot3(ctrs(:,1),ctrs(:,2),ctrs(:,3),'ko')
    xlabel('Y')
    ylabel('Cb')
    zlabel('Cr')
    axis vis3d
    % determine mean and std for each color
    % with 1.1 scale on CR
end

%% Test to see which points are in blue cluster - 2 in this case.
% figure
% k = 11;
% img = imread(FileName(k).name);
% imgsmall = imresize(img,scale);
% imshow(imgsmall);
% idx_img1 = idx(1+19200*(k-1):length(idx)/50 + 19200*(k-1));
% idx_red = 1:length(idx)/50;
% idx_red(idx_img1~=2) = [];
% [row,col] = ind2sub([120,160],idx_red);
% hold on
% plot(col,row,'ok')

%% Compute probability statistics

% let first cluster be the one with the largest cr mean:
[~,idxCr] = sort(ctrs(:,3),'descend');
mu = ctrs(idxCr,:);
idxMapped = zeros(length(idx),1);
for i = 1:k
   idxMapped(idx==idxCr(i)) = i;
end
% initialize covariance matrix
A = cell(k,1);
N = hist(idxMapped,[1:k]);
for i = 1:k
    diffmat = zeros(3,3);
    colorIdx = 1:length(idx);
    colorIdx(idxMapped ~= i) = [];
    %for j = 1:N(i)
        %diffmat = diffmat + [XscaledData(colorIdx(j),:) - mu(i,:)]'*[XscaledData(colorIdx(j),:) - mu(i,:)];
    %end
    %A{i} = inv(1/N(i)*diffmat);
    A{i} = inv(cov(XscaledData(colorIdx,:))); % inverse covariance matrix
   %keyboard
end

% compute probability of each color
Pcl = N/length(idx);


