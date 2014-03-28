function GridCoordrc = mmToGrid(WorldCoordmm,res,offset)

% Convert mm to grid of a specific size as defined here
% Input 
% WorldCoordmm : [2 x n] = [X;Y]
% res: [x;y]: mm per grid
% offset: [row,col] -> [x;y]: 0,0 -> [Ox,Oy] in grids
% Output
% WorldCoordGrid : [2 x n] 
% 

xres = res(1); %mm per grid
yres = res(2); %mm per grid

GridCoordxy = round(bsxfun(@rdivide,WorldCoordmm(1:2,:),[xres;yres]));

GridCoordxy = bsxfun(@plus,GridCoordxy,flipud(offset));

% go from [x,y] -> [rows,columns]
%GridCoordrc = [GridCoordxy(2,:);GridCoordxy(1,:)];
GridCoordrc = GridCoordxy;