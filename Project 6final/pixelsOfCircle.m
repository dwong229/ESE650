function circlepix = pixelsOfCircle(BeadCenters,radii,limIJ)

% Given the center and radius of the circle along with size of image
% return [i,j] coordinates of pixels.
% INPUT
% center : [nx2] n-beads, [i,j]
% radii : 67pixels
% limIJ : [ixj] size of image

%BeadCenters = [50,100;400 600];
%radii = 67;

centers = bsxfun(@plus,BeadCenters,[-radii -radii]);
%limIJ = [480,640];


imax = limIJ(1);
jmax = limIJ(2);
thetaResolution = .1; 
theta=(0:thetaResolution:360)'*pi/180;

x = bsxfun(@times,radii',cos(theta));
x = bsxfun(@plus,x,100);
%x = cat(1,x,nan(1,length(radii)));
x = x(:);

y = bsxfun(@times,radii',sin(theta));
y = bsxfun(@plus,y,100);
%y = cat(1,y,nan(1,length(radii)));
y = y(:);

img = zeros(limIJ);

for i = 1:length(x);
    img(round(x(i)),round(y(i))) = 1;
end

% fill circle
%J = roifill(img,y,x);
%img = imdilate(img,'full');
perim = regionprops(img,'FilledImage');
%imagesc(perim.FilledImage)

OrigcirclePix = regionprops(perim.FilledImage,'PixelList');
OrigcirclePix = OrigcirclePix.PixelList; 
jCircle = OrigcirclePix(:,1);
iCircle = OrigcirclePix(:,2);

circlepix = [];
for beadIdx = 1:size(centers,1)
    beadpix = bsxfun(@plus,[iCircle,jCircle],centers(beadIdx,:));
    circlepix = [circlepix;beadpix];
end
%imagesc(img)
%colormap(gray)

% check pixels to see if they are in limIJ:
circlepix(any(circlepix<1,2),:) = [];
circlepix(circlepix(:,1)>imax,:) = [];
circlepix(circlepix(:,2)>jmax,:) = [];

%plot(circlepix(:,2),circlepix(:,1),'.b')
%axis equal
