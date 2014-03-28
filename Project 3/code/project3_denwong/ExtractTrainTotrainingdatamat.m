% Creates trainingdata.mat from extracting the data from imu files.


% unpack IMU data
disp('Extract training data')


% use user to choose training folder
disp('Choose folder that contains training data')
folder_name = uigetdir;


% Read folder name
folder = dir(folder_name);

% remove the . and .. by looking at files from 3 to end
train = struct('labelsIdx','','data','','label','','filename','');
train(1).labelsIdx = {};
datasetidx = 1;
h1 = figure;
for i = 3:length(folder)
    
    % look in each training file to extract training data
    strdir = strcat(folder_name,'/',folder(i).name);
    imufiles = dir(strdir);
    
    disp(folder(i).name)
    
    % add labelname to labelsIdx
    train(1).labelsIdx = cat(1,train(1).labelsIdx,folder(i).name)
        
    % for each folder check if there are imu files
    for n = 3:length(imufiles)
        
        disp(imufiles(n).name)
        % save training data
        imufilename = strcat(strdir,'/',imufiles(n).name)
        
        tempdata = load(imufilename);
        timedata = tempdata(:,1) - tempdata(1,1);
        % extract IMU data
        train(datasetidx).data = [timedata tempdata(:,2:7)];
        
        % save level:
        train(datasetidx).label = i-2; % idx corresponding to labelsIdx
        
        % save filename
        train(datasetidx).filename = imufiles(n).name; % idx corresponding to labelsIdx
        
        
        % index dataset counter
        datasetidx = datasetidx + 1;
        
        % plot to visualize
        if true
            figure(h1)
            subplot(2,3,n-2)
            hold all
            for m = 2:7
                plot(timedata, tempdata(:,m))
            end
            xlabel('time')
            ylabel('IMU data')
            
        end % end plot
    end
    % end cycle through that label 
    legend('Ax','Ay','Az','Wz','Wy','Wz')
    disp('~~~~')
    keyboard
    close all
end
