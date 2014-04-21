function [map,protein] = generateProteinMap(nproteinsites,limIJ)

% Given number of protein sites and size of map generate random location
% and probability map of protein.

imax = limIJ(1);
jmax = limIJ(2);

protein.center = floor(bsxfun(@times,rand(nproteinsites,2),[imax,jmax]));
%protein.sigma = ceil(rand(nproteinsites,1)*5);
protein.sigma = ceil(ones(nproteinsites,1)*100);

map = zeros(imax,jmax);

% populate map with probability of detecting protein:
for proIdx = 1:nproteinsites
    mean = protein.center(proIdx,:);
    Sigma = eye(2)*protein.sigma(proIdx);
    
    %mean = [450,600];
    %Sigma = eye(2)*100;
    
    buffer = 200;
    x1 = mean(1)-buffer:mean(1) + buffer; %i
    x2 = mean(2)-buffer:mean(2) + buffer; %j
    
    % remove points that are out of image limits
    x1(x1>imax) = [];
    x1(x1<1) = [];
    x2(x2>jmax) = [];
    x2(x2<1) = [];
    
    [X2,X1] = meshgrid(x2,x1);
    F = mvnpdf([X2(:) X1(:)],fliplr(mean),Sigma);
    F = reshape(F,length(x2),length(x1));
    %surf(x1,x2,F); 
           
    idx = sub2ind([imax,jmax],X1(:),X2(:));

    % add probability scale factor:
    pscale = 1e6;
    map(idx) = map(idx) + F(:)*pscale;
end

