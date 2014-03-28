function rawdata = ExtractTestIMUData(folder_name)

% Read folder name
imufiles = dir(folder_name);

rawdata = struct('data','','filename','','classification','');

numfiles = length(imufiles);
datasetidx = 1;

fprintf('%2.0f test files found \n',length(imufiles))
h1 = figure
for n = 3:length(imufiles)
    % assuming 1 and 2 are . and .. CHECK THIS!
    strdir = strcat(folder_name,'/',imufiles(n).name)
    disp(strdir)
    % save training data
    
    %imufilename = strcat(strdir,'/',imufiles(n).name)
    tempdata = load(strdir);
    timedata = tempdata(:,1) - tempdata(1,1);
    % extract IMU data
    rawdata(datasetidx).data = [timedata tempdata(:,2:7)]'; % [7 x n]
    
    % save filename
    rawdata(datasetidx).filename = imufiles(n).name; % idx corresponding to labelsIdx
    
    
    % index dataset counter
    datasetidx = datasetidx + 1;
    
    
    % plot to visualize
    if true
        disp('Make sure there are no time discrepancies')
        figure(h1)
        subplot(2,3,n-2)
        hold all
        for m = 2:7
            plot(timedata, tempdata(:,m))
        end
        xlabel('time')
        ylabel('IMU data')
        % end cycle through that label
        %legend('Ax','Ay','Az','Wz','Wy','Wz')
        disp('~~~~')
        keyboard
    end % end plot
end

close all

disp('All files extracted')