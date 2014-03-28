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
        q = Quaternion(rots(1:4,i));
        R = q.R;
            th(:,i) = tr2rpy(R);
    end
end
    