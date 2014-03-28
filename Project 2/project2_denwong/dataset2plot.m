function [] = dataset2plot(tI,dataset,xlabelstr,ylabelstr,titlestr)


%figure(fighandle) 

plot(tI,dataset(1,:),'.r')
hold on
plot(tI,dataset(2,:),'.g')
plot(tI,dataset(3,:),'.b')
xlabel(xlabelstr)
ylabel(ylabelstr)
title(titlestr)
legend('X','Y','Z')
