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

%Find the Vulintus Board Definitons directory.
def_dir = which('Vulintus_Synch_Board_Definitions.m');                      %Grab the full path of the present function to find the board definitions directory.
if isempty(def_dir)                                                         %If the no directory was found...
    error('VULINTUS_SYNCH_BOARD_DEFINITIONS:BADPATH',...
        'ERROR IN VULINTUS_SYNCH_BOARD_DEFINITIONS: %s',...
        'Cound not locate "Vulintus_Synch_Board_Definitions.m"!');          %Return an error.
end
[def_dir, cur_dir, ~] = fileparts(def_dir);                                 %Strip out the filename from the path.
while ~strcmpi(cur_dir,'Vulintus Board Definitions') && ~isempty(cur_dir)   %Loop until we get to the "Vulintus Board Definitions" folder.
    [def_dir, cur_dir, ~] = fileparts(def_dir);                             %Strip out the current directory from the path.
end
if isempty(def_dir)                                                         %If the "Vulintus Board Definitions" folder wasn't found...
    error('VULINTUS_SYNCH_BOARD_DEFINITIONS:BADPATH',...
        'ERROR IN VULINTUS_SYNCH_BOARD_DEFINITIONS: %s',...
        ['Cound not locate "Vulintus Board Definitions" directory in '...
        'the script path!']);                                               %Return an error.
end
def_dir = fullfile(def_dir,'Vulintus Board Definitions');                   %Add the "Vulintus Board Definitions" folder back to the definitions path.

%Find the Arduino packages directory in the AppData folder.
local = winqueryreg('HKEY_CURRENT_USER',...
        ['Software\Microsoft\Windows\CurrentVersion\' ...
        'Explorer\Shell Folders'],'Local AppData');                         %Grab the local application data directory.  
appdata_dir = fullfile(local,'Arduino15','packages','vulintus');            %Create the expected directory for the vulintus board definitions.
if ~exist(appdata_dir,'dir')                                                %If the directory doesn't exist...
    answer = questdlg(['The Vulintus board definitions aren''t yet '...
        'installed in Arduino. Would you like to copy them from the '...
        'repository?'],'Copy Repository?','YES','NO','NO');                 %Ask the user if they want to copy the repository definitions into Arduino.
    if ~strcmpi(answer,'yes')                                               %If the user clicked anything besides "YES"...
        return                                                              %Skip execution of the rest of the function.
    end
    source_path = def_dir;                                                  %Set the source to the repository.
    target_path = appdata_dir;                                              %Set the target to the packages directory.
else                                                                        %Otherwise, if the directory already exists...
%     if exist(fullfile(appdata_dir,'hardware'),'dir') && ...
%             exist(fullfile(appdata_dir,'tools'))                            %If the "hardware" and "tools" folders already exist...
%         answer = questdlg(['Which directory is the source and which is '...
%             'the target?'],'Synchronization Direction',...
%             'Arduino Packages (Source) >> Repository (Target)',...
%             'Repository (Source) >> Arduino Packages (Target)',...
%             'Arduino Packages (Source) >> Repository (Target)');            %Ask the user which direction they want to synchronize.
%         switch answer                                                       %Switch between the possible responses...
%             case 'Arduino Packages (Source) >> Repository (Target)'         %If the packages directory is the source and the repository is the target...
%                 source_path = appdata_dir;                                  %Set the source to the packages directory.
%                 target_path = def_dir;                                      %Set the target to the repository.
%             case 'Repository (Source) >> Arduino Packages (Target)'         %If the respository is the source and the packages directory is the target...
                source_path = def_dir;                                      %Set the source to the repository.
                target_path = appdata_dir;                                  %Set the target to the packages directory.
%             otherwise                                                       %Otherwise...
%                 return                                                      %Skip execution of the rest of the function.
%         end
%     else                                                                    %Otherwise...
%         source_path = def_dir;                                              %Set the source to the repository.
%         target_path = appdata_dir;                                          %Set the target to the packages directory.
%     end
end

folders = {fullfile(source_path,'hardware');...
    fullfile(source_path,'tools')};                                         %Set the source folders to search in.
source_files = file_miner(folders,[]);                                      %Find all files in the source path.

folders = {fullfile(target_path,'hardware');...
    fullfile(target_path,'tools')};                                         %Set the target folders to search in.
target_files = file_miner(folders,[]);                                      %Find all files in the target path.

waitbar = big_waitbar('title','Synchronizing files...',...
    'color','r');                                                           %Create a waitbar figure.
wait_step = ceil(length(source_files)/100);                                 %Find an even stepsize for displaying progress on the waitbar.
for i = 1:length(source_files)                                              %Step through each source file.
    [file_source_path, short_file, ext] = fileparts(source_files{i});       %Grab the file's path.
    if waitbar.isclosed()                                                   %If the user closed the waitbar figure...
        return                                                              %Skip execution of the rest of the function.
    elseif rem(i-1,wait_step) == 0                                          %Otherwise, if it's time to update the waitbar...
        waitbar.string(sprintf('Copying: %s%s',short_file,ext));            %Update the waitbar text.
        waitbar.value(i/length(source_files));                              %Update the waitbar value.
        drawnow;                                                            %Update the plot immediately.
    end    
    file_source_path(1:length(source_path)) = [];                           %Pull the source path out of the file path.
    file_target_path = fullfile(target_path,file_source_path);              %Add the remaining subfolders to the target path.
    if ~exist(file_target_path,'dir')                                       %If the expected target path doesn't exist...
        mkdir(file_target_path);                                            %Create it.
    end
    target_file = fullfile(file_target_path,[short_file ext]);              %Create the expected target file name.
    copyfile(source_files{i},target_file,'f');                              %Copy the file to the target directory.
end

waitbar.title('Checking for deleted files...');                             %Update the title on the waitbar.
wait_step = ceil(length(target_files)/100);                                 %Find an even stepsize for displaying progress on the waitbar.
for i = 1:length(target_files)                                              %Step through each target file.
    [file_target_path, short_file, ext] = fileparts(target_files{i});       %Grab the file's path.
    if waitbar.isclosed()                                                   %If the user closed the waitbar figure...
        return                                                              %Skip execution of the rest of the function.
    elseif rem(i-1,wait_step) == 0                                          %Otherwise, if it's time to update the waitbar...
        waitbar.string(sprintf('Checking: %s%s',short_file,ext));           %Update the waitbar text.
        waitbar.value(i/length(target_files));                              %Update the waitbar value.
        drawnow;                                                            %Update the plot immediately.
    end    
    file_target_path(1:length(target_path)) = [];                           %Pull the target path out of the file path.
    file_source_path = fullfile(source_path,file_target_path);              %Add the remaining subfolders to the target path.
    possible_source_file = fullfile(file_source_path,[short_file ext]);     %Create the expected target file name.
    if ~any(strcmpi(source_files,possible_source_file))                     %If the target file doesn't match any file in the source...
        fprintf(1,'DELETING: %s\n',target_files{i});                        %Show the user which files are being deleted.
        delete(target_files{i});                                            %Delete it.
    end
    [target_files{i}, ~, ~] = fileparts(target_files{i});                   %Strip the filename out of the path.
end

target_files = unique(target_files);                                        %Find all unique path names.
file_n_char = zeros(size(target_files));                                    %Create a matrix to count characters in the path names.
for i = 1:length(target_files)                                              %Step through each target file.
    file_n_char(i) = length(target_files{i});                               %Grab the number of characters in each path name.
end
[~,i] = sort(file_n_char,'descend');                                        %Sort by number of characters in descending order.
target_files = target_files(i);                                             %Sort the paths by path length, in descending order.
waitbar.title('Checking for empty folders...');                             %Update the title on the waitbar.
wait_step = ceil(length(target_files)/100);                                 %Find an even stepsize for displaying progress on the waitbar.
for i = 1:length(target_files)                                              %Step through each target file.
    if waitbar.isclosed()                                                   %If the user closed the waitbar figure...
        return                                                              %Skip execution of the rest of the function.
    elseif rem(i-1,wait_step) == 0                                          %Otherwise, if it's time to update the waitbar...
        waitbar.string(sprintf('Checking: %s',target_files{i}));            %Update the waitbar text.
        waitbar.value(i/length(target_files));                              %Update the waitbar value.
        drawnow;                                                            %Update the plot immediately.
    end
    contents = dir(target_files{i});                                        %Grab the contents of the folder.
    checker = 1;                                                            %Reset a checker variable.
    for f = 1:length(contents)                                              %Step through the contents...
        if ~all(contents(f).name == '.') || contents(f).isdir == 1          %If the item isn't a system folder...
            checker = 0;                                                    %Set the checker variable to zero.
        end
    end
    if checker == 1                                                         %If the folder was completely empty...
        fprintf(1,'DELETING: %s\n',target_files{i});                        %Show the user which files are being deleted.
        rmdir(target_files{i});                                             %Delete it.
    end
end

waitbar.close();                                                            %Close the waitbar.