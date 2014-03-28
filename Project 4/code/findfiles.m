function [edata,imudata,hdata,kdata,kinectModel] = findfiles(allfiles,file_num)

% Given all the files in the data folder and the file number, return all
% the availble data of that number

% extract all filenames so they are search able
filenameCell = {allfiles.name}';

% look for all files containing the file_num
idx = cellfun(@(x) any(strfind(x,num2str(file_num))),filenameCell);
numfilesname = {allfiles(idx).name}';


strdata = {'Encoder','Hokuyo','imu','kinect'};

% initiate data files
edata = [];
imudata=[];
hdata = [];
kdata = [];
kinectModel = false;
for i = 1:length(strdata)
    
    % look for idx of this file type
    fileidx = cellfun(@(x) any(strfind(x,strdata{i})),numfilesname);
    %fileidx = cell2mat(fileidx);
    % if it exists, load data into appropriate filename
    if find(fileidx)
        % filename
        filename = numfilesname(fileidx);
        disp(filename)
        
        % unpack data files
        if i ==1
            edata = load(filename{1});
        elseif i==2
            hdata = load(filename{1});
        elseif i==3
            imudata = load(filename{1});
        else
            disp('Not loading kinetdata to save time')
            kinectModel = true;
            %kdata = load(filename{1});
        end
        
    else
        % no file found for this data type
        disp(strcat('No file found for: ', strdata{i},num2str(file_num)))
    end
end