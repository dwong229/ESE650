%% Protein work simulation

%
%
%

% Use [i,j] = [row,col] = [y,x] coordinates

clear all
close all

plotproteinmap = true;
makemovie = false;
% initialize workspace:
imax = 480;
jmax = 640;
map = struct('cost',ones(imax,jmax),'protein',zeros(imax,jmax),'belief',zeros(imax,jmax));

hAll = figure('Position',[184 24 1209 739]);

if makemovie
    timestep = 1/10;
    movieObj = VideoWriter('PushToRight.avi');
    open(movieObj)
end

%% Generate circular regions of protein
% model as Gaussian to determine when bead will stick
nproteinsites = 1;

%[map.protein,protein] = generateProteinMap(nproteinsites,[imax,jmax]);

map.protein(:,500:end) = 1;

if plotproteinmap
    hprotein = subplot(2,2,1);
    hprotein = imagesc(map.protein);
    title('Protein Map')
    colormap(gray)
    colorbar
    axis image
    
    %hold on
    %plot(protein.center(:,2),protein.center(:,1),'xr','MarkerSize',10)
end

%% Generate belief map
subplot(2,2,2);
hbelief = imagesc(map.belief);
title('Protein Probability Map')
colormap(gray)
colorbar
axis image


%% Generate bead location
beadradii = 15;
nbead = 5;
bead = generateBead(nbead,beadradii,[imax,jmax]);
hbeads = subplot(2,2,3);
axis ij
hcircles = viscircles(fliplr(bead.center),bead.radii,'EdgeColor','b');
hold on
%plot(bead.center(:,2),bead.center(:,1),'xr')
axis image
axis equal
axis([1 jmax 1 imax])

%% Allow user to put down robot position and orientation
title('Choose center of robot')
for i = 1:2
    if i == 2
        title('Click orientation of robot relative to center')
    end
    [x(i),y(i)] = ginput(1);
    
    x(i) = round(x(i));
    y(i) = round(y(i));
    
    hold on
    g(i) = plot(x(i),y(i),'xr');
    drawnow
end
g(3) = line(x,y);
unitVec = diff([x',y'],1)/norm(diff([x',y'],1));
th = atan2(unitVec(2),unitVec(1));

state = [y(1) x(1) th]; %[i,j,th]
% display beads and place robot:

robotposn = drawrobot(state);

delete(g)

hold on
h(2) = fill(robotposn(2,:),robotposn(1,:),'b');



% update costmap to add buffer of norm([30,20]) = 36.06

%% Select exploration mode
mode = 1;

switch mode
    
    case 1
        disp('Mode 1: Push all beads right')
        
        %% initialize plots
        subplot(2,2,3)
        hold on
        htraj = plot(0,0,':k');
        title('Trajectory to get first bead')
        subplot(2,2,4)
        hCost = imagesc(map.cost,[1 100]);
        colormap(gray)
        axis image
        title('Cost Map')
        
        
        for beadIdx = 1:nbead
            %keyboard
            disp(strcat('Bead:',num2str(beadIdx)))
            %% Plan trajectory and execute
            
            % Pick a bead to go to:
            gotobead = beadIdx;
            
            % COSTMAP: Update costmap where beads are:
            % inflate cost of region around beads to radii + 37 so that robot
            % does not interfere with beads:
            
            beadobstacles = 1:nbead;
            %beadobstacles(beadobstacles == gotobead) = [];
            radii = 67;
            
            % beads pixel list
            beadList = pixelsOfCircle(bead.center(beadobstacles,:),radii,[imax,jmax]);
            beadListInd = sub2ind([imax,jmax],beadList(:,1),beadList(:,2));
            
            map.cost = ones(imax,jmax);
            map.cost(beadListInd) = 100;
            set(hCost,'CData',map.cost)
            
            % TRAJECTORY PLANNING: Go to cell1:
            startpt = [state(1),state(2)];
            
            % Determine bead pushing angle
            pushTh = 0;
            d = sqrt(beadradii^2 - 10^2) + 20;
            goalpt = round(bead.center(gotobead,:) - d*[sin(pushTh),cos(pushTh)]);
            TrajNum = 1;
            [rowp{TrajNum},colp{TrajNum}] = runDijkstraPlanner(startpt,goalpt,map.cost,'false');
            
            set(htraj,'XData',colp{TrajNum},'YData',rowp{TrajNum})
            
            %% Visualization of robot going to first bead
            robotatgoal = false;
            TrajIdx = 2;
            while ~robotatgoal
                % move robot to next [rowp,colp]
                
                state = [rowp{1}(TrajIdx),colp{1}(TrajIdx) atan2(diff(rowp{1}(TrajIdx-1:TrajIdx)),diff(colp{1}(TrajIdx-1:TrajIdx)))];
                robotposn = drawrobot(state);
                
                set(h(2),'XData',robotposn(2,:),'YData',robotposn(1,:));
                
                robotatgoal =  TrajIdx == length(rowp{1});
                TrajIdx = TrajIdx + 1;
                %drawnow
                if makemovie
                    frame = getframe(hAll);
                    %open(movieObj)
                    writeVideo(movieObj,frame);
                end
            end
            
            state(3) = pushTh;
            robotposn = drawrobot(state);
            
            set(h(2),'XData',robotposn(2,:),'YData',robotposn(1,:));
            
            
            %% Update cost map to remove target bead
            beadobstacles(beadobstacles == gotobead) = [];
            
            % beads pixel list
            beadList = pixelsOfCircle(bead.center(beadobstacles,:),radii,[imax,jmax]);
            beadListInd = sub2ind([imax,jmax],beadList(:,1),beadList(:,2));
            
            map.cost = ones(imax,jmax);
            map.cost(beadListInd) = 100;
            set(hCost,'CData',map.cost)
            
            %% Push Bead Right
            
            startpt = [state(1) state(2)];
            goalpt = [state(1) jmax-1];
            TrajNum = 2;
            [rowp{TrajNum},colp{TrajNum}] = runDijkstraPlanner(startpt,goalpt,map.cost,'false');
            
            % update traj:
            set(htraj,'XData',colp{TrajNum},'YData',rowp{TrajNum})
            TrajIdx = 2;
            
            subplot(2,2,3)
            robotatgoal = false;
            while ~bead.stuck(beadIdx) && ~robotatgoal
                
                
                % update position
                % move robot to next [rowp,colp]
                state = [rowp{TrajNum}(TrajIdx),colp{TrajNum}(TrajIdx) atan2(diff(rowp{TrajNum}(TrajIdx-1:TrajIdx)),diff(colp{TrajNum}(TrajIdx-1:TrajIdx)))];
                % update plot
                robotposn = drawrobot(state);
                % robot
                set(h(2),'XData',robotposn(2,:),'YData',robotposn(1,:));
                % bead
                bead = updatePushedBead(state,bead,beadIdx);
                updateBeadPlot(bead,hcircles)
                
                % sample protein
                % protein detected:
                
                if rand(1) < map.protein(bead.center(beadIdx,1),bead.center(beadIdx,2))
                    bead.stuck(beadIdx) = true;
                    disp('bead stuck: Protein detected!')
                    subplot(2,2,3)
                    plot(bead.center(beadIdx,2),bead.center(beadIdx,1),'xr')
                    % update protein map
                    map.belief = updateProteinMap(bead.center(beadIdx,:),map.belief,[imax,jmax],bead.stuck(beadIdx));
                    set(hbelief,'CData',map.belief);
                else
                    map.belief = updateProteinMap(bead.center(beadIdx,:),map.belief,[imax,jmax],bead.stuck(beadIdx));
                end
                
                drawnow
                % check robotat goal and update TrajIdx
                robotatgoal =  TrajIdx == length(rowp{TrajNum});
                TrajIdx = TrajIdx + 1;
                
                if makemovie
                    frame = getframe(hAll);
                    %open(movieObj)
                    writeVideo(movieObj,frame);
                end
            end
            
        end % bead for-loop
        if makemovie
            close(movieObj)
        end
        
    otherwise
        disp('Mode value invalid')
end
