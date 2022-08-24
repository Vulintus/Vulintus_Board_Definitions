function Vulintus_Synch_Board_Definitions

%
%Vulintus_Synch_Board_Definitions.m - Vulintus, Inc., 2022.
%
%   VULINTUS_SYNCH_BOARD_DEFINITIONS copies all Vulintus firmware
%   libraries from the Github- and Google Drive-backed working folder to
%   the user's local Arduino libraries folder, typically located at:
%
%   C:\Users\[USERNAME]\Documents\Arduino\libraries
%
%   UPDATE LOG:
%   05/24/2022 - Drew Sloan - Function first created.
%

close all force;                                                            %Close all open figures.

ext = {'*.h','*.cpp','*.ino','*.m','*.txt'};                                %List the file extensions to copy.

%Set the target directories.
common_dir = which('Vulintus_Update_Local_Firmware_Libraries');             %Grab the full path of the present function to find the common functions directory.
[common_dir, cur_dir, ~] = fileparts(common_dir);                           %Strip out the filename from the path.
while ~strcmpi(cur_dir,'Vulintus Common MATLAB Scripts') && ...
        ~isempty(cur_dir)                                                   %Loop until we get to the "Vulintus Software" folder.
    [common_dir, cur_dir, ~] = fileparts(common_dir);                       %Strip out the filename from the path.
end
if isempty(cur_dir)                                                         %If the "Vulintus Software" folder wasn't found...
    error('VULINTUS_UPDATE_LOCAL_FIRMWARE_LIBRARIES:BADPATH',...
        'ERROR IN VULINTUS_UPDATE_LOCAL_FIRMWARE_LIBRARIES: %s',...
        ['Cound not locate "Vulintus Common MATLAB Scripts" directory '...
        'in the script path!']);                                            %Return an error.
end
library_dir = fullfile(common_dir,'Vulintus Firmware Libraries');           %Set the expected path for the OmniTrak Arduino libraries.
if ~exist(library_dir,'dir')                                                %If the Vulintus Firmware Libraires directory doesn't exist...
    error('VULINTUS_UPDATE_LOCAL_FIRMWARE_LIBRARIES:BADPATH',...
        'ERROR IN VULINTUS_UPDATE_LOCAL_FIRMWARE_LIBRARIES: %s',...
        'Cound not locate the "Vulintus Firmware Libraries" directory!');   %Return an error.
end
local_dir = fullfile(getenv('USERPROFILE'),'\Documents\Arduino\libraries'); %Create the expected path to the Arduino libraries in the user's Documents folder.
if ~exist(local_dir,'dir')                                                  %If the Arduino libraries directory doesn't exist...
    error('VULINTUS_UPDATE_LOCAL_FIRMWARE_LIBRARIES:BADPATH',...
        'ERROR IN VULINTUS_UPDATE_LOCAL_FIRMWARE_LIBRARIES: %s',...
        'Cound not locate "\Documents\Arduino\libraries" directory!');      %Return an error.
end

waitbar = big_waitbar('title','Finding all firmware subfolders...',...
    'color','r');                                                           %Create a waitbar figure.
folders = {library_dir};                                                    %Convert the directories input to a cell array.
checker = zeros(1,length(folders));                                         %Create a checking matrix to see if we've looked in all the subfolders.
while any(checker == 0)                                                     %Keep looking until all subfolders have been checked for *.xls files.
    a = find(checker == 0,1,'first');                                       %Find the next folder that hasn't been checked for subfolders.
    temp = dir(folders{a});                                                 %Grab all the files and folders in the current folder.
    if waitbar.isclosed()                                                   %If the user closed the waitbar figure...
        return                                                              %Skip execution of the rest of the function.
    else                                                                    %Otherwise...
        [~, cur_dir] = fileparts(folders{a});                               %Grab the current directory.
        waitbar.string(sprintf('Checking: %s',cur_dir));                    %Update the waitbar text.
        waitbar.value(sum(checker)/length(checker));                        %Update the waitbar value.
    end
    for f = 1:length(temp)                                                  %Step through all of the returned contents.
        if ~any(temp(f).name == '.') && temp(f).isdir == 1                  %If an item is a folder, but not a system folder...
            subfolder = fullfile(folders{a},temp(f).name);                  %Concatenate the full subfolder name...
            if ~any(strcmpi(subfolder,folders))                             %If the subfolder is not yet in the list of subfolders...                
                folders{end+1} = subfolder;                                 %Add the subfolder to the list of subfolders.
                checker(end+1) = 0;                                         %Add an entry to the checker matrix to check this subfolder for more subfolders.
            end
        end
    end
    checker(a) = 1;                                                         %Mark the last folder as having been checked.        
end

N = 0;                                                                      %Create a file counter.
for i = 1:length(ext)                                                       %Step through each search string.
    waitbar.title(['Counting ' ext{i} ' files...']);                        %Update the title on the waitbar.
    for f = 1:length(folders)                                               %Step through every subfolder.
        temp = dir(fullfile(folders{f},ext{i}));                            %Grab all the matching filenames in the subfolder.
        N = N + length(temp);                                               %Add the number of files to the file counter.
        if waitbar.isclosed()                                               %If the user closed the waitbar figure...
            return                                                          %Skip execution of the rest of the function.
        else                                                                %Otherwise...
            [~, cur_dir] = fileparts(folders{a});                           %Grab the current directory.
            waitbar.string(sprintf('Checking: %s',cur_dir));                %Update the waitbar text.
            waitbar.value(f/length(folders));                               %Update the waitbar value.
        end
    end
end

files = cell(N,1);                                                          %Create an empty cell array to hold filenames.
file_times = zeros(N,1);                                                    %Create a matrix to hold file modification dates.
wait_step = ceil(length(files)/100);                                        %Find an even stepsize for displaying progress on the waitbar.
N = 0;                                                                      %Reset the file counter.
for i = 1:length(ext)                                                       %Step through each search string.
    waitbar.title(['Saving ' ext{i} ' filenames...']);                      %Update the title on the waitbar.
    for f = 1:length(folders)                                               %Step through every subfolder.
        temp = dir(fullfile(folders{f},ext{i}));                            %Grab all the matching filenames in the subfolder.
        for j = 1:length(temp)                                              %Step through every matching file.
            if ~all(temp(j).name == '.')                                    %If the filename isn't a hidden folder.
                N = N + 1;                                                  %Increment the file counter.
                files{N} = fullfile(folders{f},temp(j).name);               %Save the filename with it's full path.
                file_times(N) = temp(j).datenum;                            %Save the last file modification date.
                if waitbar.isclosed()                                       %If the user closed the waitbar figure...
                    return                                                  %Skip execution of the rest of the function.
                elseif rem(N,wait_step) == 0                                %Otherwise, if it's time to update the waitbar...
                    waitbar.string(sprintf('Indexing: %s',temp(j).name));   %Update the waitbar text.
                    waitbar.value(N/length(files));                         %Update the waitbar value.
                    drawnow;                                                %Update the plot immediately.
                end
            end
        end
    end
end

waitbar.title('Copying files...');                                          %Update the title on the waitbar.
for i = 1:length(files)                                                     %Step through each file.
    [file_path, short_file, exten] = fileparts(files{i});                   %Grab the file's path.
     if waitbar.isclosed()                                                  %If the user closed the waitbar figure...
        return                                                              %Skip execution of the rest of the function.
    elseif rem(N,wait_step) == 0                                            %Otherwise, if it's time to update the waitbar...
        waitbar.string(sprintf('Copying: %s%s',short_file,exten));          %Update the waitbar text.
        waitbar.value(i/length(files));                                     %Update the waitbar value.
        drawnow;                                                            %Update the plot immediately.
    end
    file_path(1:length(library_dir)) = [];                                  %Kick out the first part of the path up to Vulintus Firmware Libraries.
    target_path = fullfile(local_dir,file_path);                            %Create the expected local path name.
    if ~exist(target_path,'dir')                                            %If the expected local path doesn't exist...
        mkdir(target_path);                                                 %Create it.
    end
    local_file = fullfile(target_path,[short_file exten]);                  %Create the expected local file name.
    info = dir(local_file);                                                 %Grab any info about the file if it already exists.
    if ~isempty(info)                                                       %If the file is already in the local folder...
        if info.datenum > file_times(i)                                     %If the existing file is more recent than the one to be copied...
            fprintf(1,'LOCAL FILE IS MORE RECENT: %s%s\n',shortfile,exten); %Print a message to the serial line.
            continue                                                        %Skip to the next file.
        end
    end
    copyfile(files{i},local_file,'f');                                      %Copy the file to the local directory.
end

waitbar.title('Checking for obsolete local folders...');                    %Update the title on the waitbar.
folders = dir(fullfile(local_dir,'Vulintus*'));                             %Grab all folders in the Vulintus directory.
folders([folders.isdir] == 0) = [];                                         %Kick out any non-folders.
folders = {folders.name}';                                                  %Convert the folders to a cell array.
for f = 1:length(folders)                                                   %Step through each folder.
    folders{f} = fullfile(local_dir,folders{f});                            %Add the local Documents folder to the overall folder path.
end
checker = zeros(1,length(folders));                                         %Create a checking matrix to see if we've looked in all the subfolders.
while any(checker == 0)                                                     %Keep looking until all subfolders have been checked for *.xls files.
    a = find(checker == 0,1,'first');                                       %Find the next folder that hasn't been checked for subfolders.
    temp = dir(folders{a});                                                 %Grab all the files and folders in the current folder.
    if waitbar.isclosed()                                                   %If the user closed the waitbar figure...
        return                                                              %Skip execution of the rest of the function.
    else                                                                    %Otherwise...
        [~, cur_dir] = fileparts(folders{a});                               %Grab the current directory.
        waitbar.string(sprintf('Checking: %s',cur_dir));                    %Update the waitbar text.
        waitbar.value(sum(checker)/length(checker));                        %Update the waitbar value.
    end
    for f = 1:length(temp)                                                  %Step through all of the returned contents.
        if ~any(temp(f).name == '.') && temp(f).isdir == 1                  %If an item is a folder, but not a system folder...
            subfolder = fullfile(folders{a},temp(f).name);                  %Concatenate the full subfolder name...
            if ~any(strcmpi(subfolder,folders))                             %If the subfolder is not yet in the list of subfolders...                
                folders{end+1} = subfolder;                                 %Add the subfolder to the list of subfolders.
                checker(end+1) = 0;                                         %Add an entry to the checker matrix to check this subfolder for more subfolders.
            end
        end
    end
    checker(a) = 1;                                                         %Mark the last folder as having been checked.        
end
checker = ones(1,length(folders));                                          %Create a checking matrix to see if we've looked in all the subfolders.
for f = 1:length(folders)                                                   %Step through each folder.
    target_path = folders{f};                                               %Grab the folder name.
    target_path(1:length(local_dir)) = [];                                  %Kick out the first part of the path up to Arduino libraries.
    target_path = fullfile(library_dir,target_path);                        %Create the expected libraries path name.
    if ~exist(target_path,'dir')                                            %If the expected local path doesn't exist...
        if waitbar.isclosed()                                               %If the user closed the waitbar figure...
            return                                                          %Skip execution of the rest of the function.
        else                                                                %Otherwise...
            [~, cur_dir] = fileparts(folders{f});                           %Grab the current directory.
            waitbar.string(sprintf('Deleting: %s',cur_dir));                %Update the waitbar text.
            waitbar.value(f/length(folders));                               %Update the waitbar value.
        end
        delete(folders{f});                                                 %Delete the local folder.
        checker(f) = 0;                                                     %Marker the folder for exclusion.
    end
end
folders(checker == 0) = [];                                                 %Kick out all deleted folders.

for i = 1:length(ext)                                                       %Step through each search string.
    waitbar.title('Checking for obsolete files...');                        %Update the title on the waitbar.
    for f = 1:length(folders)                                               %Step through every subfolder.
        if waitbar.isclosed()                                               %If the user closed the waitbar figure...
            return                                                          %Skip execution of the rest of the function.
        else                                                                %Otherwise...
            [~, cur_dir] = fileparts(folders{f});                           %Grab the current directory.
            waitbar.string(sprintf('Checking: %s',cur_dir));                %Update the waitbar text.
            waitbar.value(f/length(folders));                               %Update the waitbar value.
        end
        temp = dir(fullfile(folders{f},ext{i}));                            %Grab all the matching filenames in the subfolder.
        for j = 1:length(temp)                                              %Step through every matching file.            
            if ~all(temp(j).name == '.')                                    %If the filename isn't a hidden folder.
                file_path = folders{f};                                     %Grab the file's path.
                file_path(1:length(local_dir)) = [];                        %Kick out the first part of the path up to Vulintus Firmware Libraries.
                target_path = fullfile(library_dir,file_path);              %Create the expected local path name.
                library_file = fullfile(target_path,temp(j).name);          %Create the expected local file name.
                if ~exist(library_file,'file')                              %If the local file doesn't exist in the library file...
                    delete(fullfile(folders{f},temp(j).name));              %Delete it.
                end
            end
        end
    end
end

waitbar.close();                                                            %Close the waitbar.