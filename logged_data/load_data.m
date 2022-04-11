function old_file_data=load_data(file_str)
% ADDME Function for loading the measurement data from old system
%    file_str = A string of the filename to read should end with .ASC

fid = fopen(file_str);                      % open file
for i=1:7
    temp = fgetl(fid);                      % remove title rows
end
fscanf(fid,'%e',[13 inf]);                  % import data
old_file_data = ans';
fclose(fid);                                % close file

disp('Sample data read!')