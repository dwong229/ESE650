function [camData,TimeCam] = Cam2Data(FileName)

% INPUT
% FileName for camera data
%
% OUTPUT
% camData M*N*K, K number of images, images size M*N
% TimeCam K*1

load(FileName)

camData = cam;
TimeCam = ts;
