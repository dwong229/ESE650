function [] = dataset2plot(dataset,xlabelstr,ylabelstr)

figure(fighandle) 

plot(tI,dataset(1,:),'.r')
hold on
plot(tI,dataset(2,:),'.g')
plot(tI,dataset(3,:),'.b')
xlabel(xlabelstr)
ylabel(ylabelstr)

