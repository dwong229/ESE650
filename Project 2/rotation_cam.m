

%time = [17.135, 17.5];
time = [15.503, 16.2]

v1 = zeros(3,2);

for i = 1:2
    [~,idxV] = min(abs(tV-time(i)))
    
    [~,idxC] = min(abs(tC-time(i)))

    R = rots(:,:,idxV);
    v1(:,i) = R'*[1;0;0];
    
    img{i} = camData(:,:,:,idxC);
end
    
    
th_deg = rad2deg(acos(sum(v1(1:2,1).*v1(1:2,2))/(norm(v1(1:2,1))*norm(v1(1:2,2)))));
th_deg = rad2deg(acos(sum(v1([1,3],1).*v1([1,3],2))/(norm(v1([1,3],1))*norm(v1([1,3],2)))));

figure
subplot(1,2,1)
imshow(img{1})
subplot(1,2,2)
imshow(img{2})
title(th_deg)

figure
