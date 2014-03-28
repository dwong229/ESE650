function R = rotxyz(wx,wy,wz)

% apply rotation R = Rx Ry Rz, wx wy wz defined in radians
% uses rvctools

R = rotx(wx)*roty(wy)*rotz(wz);