disp('Acquiring File Information...')

DirName = uigetdir('Select Folder with Training Set Images');
addpath(genpath(DirName));

dirstruct = dir(strcat(DirName,'/*.png'));

fprintf('There are %2.0f test files. \n Beginning Test...\n',length(dirstruct))

for i = 1:length(dirstruct),
    disp('------')
    disp(dirstruct(i).name)
    % Current test image
    im = imread(dirstruct(i).name);
    % Your algorithm here!
    [x, y, d] = denwongRedBarrel(im);
    % Display results:
    hf = figure(1);
    set(hf,'Position',[800 132 827 597])
    image(im);
    hold on;
    plot(x, y, 'g+');
    title(sprintf('Barrel distance: %.1f m', d));
    % You may also want to plot and display other
    % diagnostic information such as the outlines
    % of connected regions, etc.
    hold off;
    pause;
end
disp('End of Test.')