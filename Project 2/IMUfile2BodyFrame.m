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
    sensitivity = varargin{2};oh 
    
else
    disp('IMU parameters NOT entered')
    sensitivity = 1;
    Vref = 1;
end

% constants
g = 9.81;
sensitivity = g/(vals(3,1) - 505);
gyro_sf = 3300/1023*pi/180/16.4*15;
gyro_sf = 1*pi/180;

Ax = -(-vals(1,:) + 511)*sensitivity;
Ay = -(-vals(2,:) + 501)*sensitivity;
Az = -(vals(3,:) - 505)*sensitivity;

Wz = (vals(4,:) -  mean(vals(4,1:5)))*gyro_sf;
Wx = (vals(5,:) - mean(vals(5,1:5)))*gyro_sf;
Wy = (vals(6,:) - mean(vals(6,1:5)))*gyro_sf;
TimeIMU = ts; % convert to seconds