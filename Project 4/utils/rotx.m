function r = rotx(t,varargin)

% Homogeneous transformation representing a rotation of theta
% about the X axis.

ct = cos(t);
st = sin(t);

if ~isempty(varargin) && varargin{1} ==3
    r =    [1	0	 0
        0	ct -st
        0	st ct];
    
    
    
else
    
    r =    [1	0	 0	 0
        0	ct -st 0
        0	st ct	 0
        0	0	 0	 1];
    
    
end