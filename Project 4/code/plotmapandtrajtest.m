%% plot traj and map for SLAM
figure;
imagesc((map.gridmap'))
% plot map in map coords

offset = map.offset + [1;1]; %[rows,col]
xy = robotstate.state(1:2,:);
xy = floor(bsxfun(@rdivide,robotstate.state(1:2,:),map.resolution));
% adjust traj coords to map coords based on offset
gridtrajx = xy(1,:) + offset(2);
gridtrajy = xy(2,:) + offset(2);

%hold on
hold on;
plot(gridtrajx,gridtrajy,'.k')
axis xy

