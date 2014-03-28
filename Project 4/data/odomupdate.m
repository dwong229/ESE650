function [q1] = odomupdate(q0,delta,w)

%% compute new location using arc length approximation of skid slip model
% INPUT
% q0: x,y,theta
% delta : encoder wheel distances
% w: Distance between wheels (mm)
% 

%% robot parameters
x0 = q0(1);
y0 = q0(2);
th0 = q0(3);

dl = delta(1,1);
dr = delta(2,1);

if dl==dr
    R = 0;
    alpha = 0;
    dx = 0;
    dy = delta(2);
    
else % compute radius from instantaneous center
    R = w/2*(dl+dr)/(dl-dr); %mm
    %alpha = w/(dl-dr);
    if dl ~= 0
        alpha = dl/(R+w/2);
    else
        alpha = dr/(R-w/2);
    end
    dx = R*(1-cos(alpha));
    dy = R*sin(alpha);
end

    
%alpha = dl/(w/2*(1+(dl+dr)/(dl-dr))); %radians

dq = [dx;dy;-alpha];
%% Linear
%A = [cos(th0)/2 cos(th0)/2;sin(th0)/2 sin(th0)/2;-1/(w) 1/(w)];
%dq = A*delta;
%q1 = q0+dq;

%%
%th0 = +th0-pi/2;
th0 = +th0;
RbodyWorld = [cos(th0) -sin(th0) 0;sin(th0) cos(th0) 0;0 0 1];
q1 = q0 + RbodyWorld*dq;

%q1(3) = mod(q1(3),2*pi);


if any(isnan(q1))
    disp('q1 NAN')
    keyboard
end
