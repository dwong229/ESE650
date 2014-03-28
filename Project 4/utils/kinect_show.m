%k = load('kinect20_unpacked_0.mat')
k = load('kinect20.mat')
%figure; plot(1:100, k.rgb_ts, 1:100, k.depth_ts) % time series
figure; imshow(squeeze(k.rgb(1,:,:,:))/255) % first RGB frame
figure; imshow(squeeze(k.depth(1,:,:))/2048) % first depth frame

