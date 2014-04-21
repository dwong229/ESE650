function [ip1,jp1] = runDijkstraPlanner(startpt,goalpt,costmap,varargin)

% Given costmap and start and goal pt, compute shortest path between them
% and plot.
% goalpt: [row,col]
% startpt:[row,col]
% costmap:

ctg = dijkstra_matrix(costmap,goalpt(1),goalpt(2)); % computes cost to go

%disp('Start dijkstra_path')
[ip1, jp1] = dijkstra_path(ctg, costmap, startpt(1), startpt(2));
%disp('End dijkstra_path')
%disp('Start dijkstra_path2')
%[ip2, jp2] = dijkstra_path2(ctg, costmap, startpt(1), startpt(2));
%disp('End dijkstra_path2')

if isempty(varargin)
    plotoption = true;
    figure;
elseif varargin{1} == 'false'
    plotoption = false;
else
    plotoption = true;
    figure(varargin{1});
end


if plotoption 
    subplot(2,1,1);
    title('Costmap')
    %imagesc(costmap,[1 10]);
    imagesc(costmap,[min(costmap(:)),max(costmap(:))])
    colormap(1-gray);
    hold on;
    %plot(jp1, ip1, 'b-', jp2, ip2, 'r-');
    plot(jp1, ip1, 'b-');
    axis image
    hold off;
    
    subplot(2,1,2);
    title('Cost to go')
    imagesc(ctg);
    colormap(1-gray);
    hold on;
    %plot(jp1, ip1, 'b-', jp2, ip2, 'r-');
    plot(jp1, ip1, 'b-');
    axis image
    hold off;
end