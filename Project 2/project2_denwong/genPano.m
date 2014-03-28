function [imgCoords] = genPano(tI,rots,tC,camData)

% given IMUrotation matrices and camera data, generate panorama


%% project the image onto a sphere

R = 1;
CameraViewingAngle = [68,47]; % degrees in image xaxis and image yaxis

yawx = CameraViewingAngle(1)/320; % degrees per pixel in image +x direction, world -y

%pitchy = 47/240; % degrees per pixel in image y direction,
pitchy = CameraViewingAngle(2)/240; % degrees per pixel in image y direction,


% imagesphere = [pitch, yaw]
%imgSphere = zeros(360/pitchy,360/yawx,3); % initializ as black pixels

imgCoords = zeros(240,320,3); % will get bigger
identityCorner = [1,1];
%% Compute yaw and pitch of camera

% for i = 1:length(tC)
%     Rb2w = rots(:,:,i)';
%     bodyx = rots(1,:); % 1st row os rots
%
%     % X Z PLANE
%     pitchangle =  rad2deg(acos(sum(bodyx([1,3],1).*[1,0])/norm(bodyx([1,3]))));
%     % X Y PLANE
%     yawangle = rad2deg(acos(sum(bodyx([1,2],1).*[1,0])/norm(bodyx([1,2]))));
%
%     % convert yaw and pitch angle to coordinates in imgSphere
%
%     % populate imgSphere with pixels
%
% end



shiftby = [0 0];
%% Compute where top left corner of image is relative to R = eye(3) in pixels
%X Z PLANE
for i = 1:1:length(tC)
    
    
    
    [~,idx] = min(abs(tI - tC(i)));
    
    Rb2w = rots(:,:,idx)';
    bodyx = Rb2w(:,1)'; % 1st row os rots
    
    bodyy = Rb2w(:,2)'; % 1st row os rots
    
    
    rollangle = abs(rad2deg(atan2(bodyy(3),bodyy(2))));
    
    if rollangle<10;
    
    %pitchangle =  rad2deg(acos(sum(bodyx([1,3]).*[1,0])/norm(bodyx([1,3]))));
    pitchangle =  rad2deg(atan2(bodyx(3),bodyx(1)));
    %X Y PLANE
    %yawangle = -rad2deg(acos(sum(bodyx([1,2]).*[1,0])/norm(bodyx([1,2]))));
    yawangle = rad2deg(atan2(bodyx(2),bodyx(1)));
    
    % corner = [row, col]
    topcornercoord = ceil([pitchangle yawangle]./[pitchy yawx]) + identityCorner;

    % check if new top corner exceeds matrix: (i.e. moving in - pitch and +
    % yaw
    if sum(topcornercoord<0)>0
        shiftby(topcornercoord<1) = abs(topcornercoord(topcornercoord<1))+1;
        identityCorner = identityCorner + shiftby;
        if shiftby(1)>0
            % add rows
            imgCoords = [zeros(shiftby(1),size(imgCoords,2),3);imgCoords];
        end
        if shiftby(2)>0
            % add columns
            imgCoords = cat(2,zeros(size(imgCoords,1),shiftby(2),3),imgCoords);
            
        end
        topcornercoord = ceil([pitchangle yawangle]./[pitchy yawx]) + identityCorner;
  
    end
    
    if sum(topcornercoord == 0)>0
        topcornercoord(topcornercoord==0) = topcornercoord(topcornercoord==0) + 1;
    end
    
    
    othercorner = topcornercoord + [239,319];
    
    imgCoords(topcornercoord(1):othercorner(1),topcornercoord(2):othercorner(2),:) = camData(:,:,:,i);
    end
end

