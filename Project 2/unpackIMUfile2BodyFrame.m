function [Ax,Ay,Az,Wx,Wy,Wz,TimeIMU] = IMUfile2BodyFrame(FileName,varargin)

% INPUT
% FileName
% Vref: varargin{1}
% sensitivity: varargin{2}
%  VARARGIN NOT YET IMPLEMENTED
% OUTPUT
% Ax Ay Az : Linear Acceleration reading from IMU in Body frame
% Wx Wy Wz : Angular Acceleration reading from IMU in Body frame

load(FileName)

if ~isempty(varargin)
    disp('IMU parameters entered')
    Vref = varargin{1};
    sensitivity = varargin{2};
    
else
    disp('IMU parameters NOT entered')
    sensitivity = 1;
    Vref = 1;
end

Ax = -vals(1,:);
Ay = -vals(2,:);
Az = vals(3,:);
Wz = vals(4,:);
Wx = -vals(5,:);
Wy = -vals(6,:);
TimeIMU = ts;