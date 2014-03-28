function R = rotzyx(w)

% apply rotation R = Rx Ry Rz, wx wy wz defined in radians
% uses rvctools
ct = cos(w);
st = sin(w);
rz =  [ct(1),-st(1),0,0;st(1),ct(1),0,0;0,0,1,0;0,0,0,1];
ry =  [ct(2),0,st(2),0;0,1,0,0;-st(2),0,ct(2),0;0,0,0,1];
rx =  [1,0   0   0;0    ct(3)	-st(3)	0;0   st(3)	ct(3)	0;0	   0	0	1];
R = rz*ry*rx;