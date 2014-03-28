function [error] = compareAxisAngle(tV,RVicon,tI,IMUrots)

% Given R, compare rotation vectors in angle axis representation

% convert R to angle axis

[angleaxisVicon] = rots2angleaxis(RVicon);
[angleaxisIMU] = rots2angleaxis(IMUrots);

angleaxisR = zeros(4,length(tI));
% sync times and compute error
for i = 1:length(tI)
    [~,vidx] = min(abs(tV-tI(i)));
    %error(i) = norm(angleaxisIMU(:,i) - angleaxisVicon(:,vidx));
    %error(i) = rad2deg(acos(sum(angleaxisIMU(:,i).*angleaxisVicon(:,vidx))));
    dR = RVicon(:,:,vidx)*IMUrots(:,:,i)';
    angleaxisR(:,i) = rots2angleaxis(dR);
    
end

error = angleaxisR(1,:);

figure
subplot(2,1,1)
plot(tI,rad2deg(angleaxisR(1,:)))
xlabel('time')
ylabel('Angle error (deg)')