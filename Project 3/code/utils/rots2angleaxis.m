function [angleaxis] = rots2angleaxis(rots)

% convert from rotation matrix to angle axis representation

n = size(rots,3);

angleaxis  = zeros(4,n);


for i = 1:n
    if ~isnan(rots(1,1,i))

    angleaxis(1,i) = acos((trace(rots(:,:,i))-1)/2);
    [V,D] = eig(rots(:,:,i));
    [~,idx] = min(diag(D) - 1);
    angleaxis(2:4,i) = V(:,idx);
end
end

