function map = updateProteinMap(mean,map,limIJ,stuck)

%
%
%

imax = limIJ(1);
jmax = limIJ(2);

% Gaussian model for detection
Sigma = eye(2)*5;

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

    if stuck
    % add probability scale factor:
        pscale = 20;
        map(idx) = map(idx) + F(:)*pscale;
    else
        pscale = -10;
        map(idx) = map(idx) + F(:)*pscale;
    end
end