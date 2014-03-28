% genPano test

%time = [15.503, 16.2, 16.6]

%v1 = zeros(3,2);

%camDatatest = zeros(240,320,3,2,'uint8');
%R = zeros(3,3,2);

% for i = 1:3
%     [~,idxV] = min(abs(tV-time(i)));
%     
%     [~,idxC(i)] = min(abs(tC-time(i)));
% 
%     R(:,:,i) = rots(:,:,idxV);
%     %v1(:,i) = R'*[1;0;0];
%     
%     camDatatest(:,:,:,i) = camData(:,:,:,idxC);
% end


%rots(1:3,1:3,1) = eye(3);
%rots(1:3,1:3,2) = rotz(5,'deg')*roty(10,'deg');

%camData = zeros(240,320,3,2);
%camData(:,:,1,1) = 1; % red
%camData(:,:,2,2) = 1; % green

time = [15,42]; %1
time = [11,38]; %2
time = [9,27] % 8

[~,idxC(1)] = min(abs(tC-time(1)));
[~,idxC(2)] = min(abs(tC-time(2)));

img = genPano(tV,RVicon,tC(idxC(1):idxC(2)),camData(:,:,:,idxC(1):idxC(2)));
img = uint8(img);
figure
imshow(img)

