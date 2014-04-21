function [] = updateBeadPlot(bead,hggbead)


% unpack figure handle
h = get(hggbead,'Children');

centers = fliplr(bead.center);
radii = bead.radii;

% compute circles:
thetaResolution = 2; 
theta=(0:thetaResolution:360)'*pi/180;

x = bsxfun(@times,radii',cos(theta));
x = bsxfun(@plus,x,(centers(:,1))');
x = cat(1,x,nan(1,length(radii)));
x = x(:);

y = bsxfun(@times,radii',sin(theta));
y = bsxfun(@plus,y,(centers(:,2))');
y = cat(1,y,nan(1,length(radii)));
y = y(:);

set(h(1),'XData',x,'YData',y);

set(h(2),'XData',x,'YData',y);

