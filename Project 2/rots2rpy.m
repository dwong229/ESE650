function [th] = rots2rpy(rots,strOption)

th = zeros(3,size(rots,3));


if strOption == 'mat'
    disp('rots2rpy: R matrix inserted')
for i = 1:size(rots,3)
    th(:,i) = tr2rpy(rots(:,:,i));
end
elseif strOption == 'quat'
    disp('rots2rpy: quaternion matrix inserted')
    for i = 1:size(rots,2)
        R = Quat2Rot(rots(1:4,i));
        th(:,i) = tr2rpy(R);
    end
end
    