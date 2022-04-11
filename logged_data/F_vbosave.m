%
%	Racelogic Save VBO file
%
function F_vbosave
global vbo;

%% test for existence
if (0 == size(vbo,1))
	error 'Load a VBO file first.';
end

%% open file for write
[filename,path] = uiputfile('*.vbo', 'Save VBO File As...');

if (isequal(filename, 0))
	return;													% cancel
end

[fid, message] = fopen( fullfile(path,filename), 'wt' );

if (-1 == fid)
	error( message );
end

%% setup vars
num_standard_channels = 0;

standard_channels = [];
run vbo_standard_channels

savetime = datestr(now, 'dd/mm/yyyy at HH:MM');

fprintf(1, 'Saving %s...\n', filename);

%% write sections
for secnum = 1:size(vbo.sections,2)

%% write out section header - unless 'created' pseudo-section.
	if (1 ~= strcmp('created', vbo.sections(secnum).name))
		fprintf(fid, '[%s]\n', vbo.sections(secnum).name);
	end

%% write the data
	switch (vbo.sections(secnum).name)
	case 'created'
		write_created;
	case 'header'
		write_header;
	case 'channel units'
		write_channel_units;
	case 'data'
		write_data;
	case 'comments'
		write_comments;
	otherwise
		write_misc;
	end
end

fclose(fid);

disp('Done.');

%% end main write function. start nested functions.

%% write [created]
% write the current time out to disk but leave the version in memory as it
% was. This makes sense to me. If the user wants comments and headers as
% saved they can reload the file.
function write_created
	fprintf(fid, 'File created on %s\n\n', savetime);
end

%% write [header]
function write_header
	for channel_num = 1:size(vbo.channels,2)
		for std_num = 1:size(standard_channels,1)
			if (1 == strcmp( ...
					standard_channels(std_num,1), ...
					vbo.channels(channel_num).name ...
				))
				num_standard_channels = num_standard_channels + 1;
				break;
			end
		end
		fprintf(fid, '%s\n', vbo.channels(channel_num).name);
	end
	fprintf(fid, '\n');									% maintain normal spacing
end

%% write [channel units]
function write_channel_units
	for channel_num = 1+num_standard_channels:size(vbo.channels,2)
		fprintf(fid, '%s\n', vbo.channels(channel_num).units);
	end
	fprintf(fid, '\n');									% maintain normal spacing
end

%% write [data]

% simple, CPU-heavy, memory-efficient at the moment

function write_data

	% work out fprintf format string
	format_string = '';
	time_channel = 0;
	
	for channel_num = 1:size(vbo.channels,2)
		switch(vbo.channels(channel_num).name)
		case 'satellites'
			format_string = [format_string '%03.0f ']; %#ok<AGROW>
		case 'time'
			format_string = [format_string '%09.2f ']; %#ok<AGROW>
			time_channel = channel_num;
		case 'latitude'
			format_string = [format_string '%+014.8f ']; %#ok<AGROW>
		case 'longitude'
			format_string = [format_string '%+015.8f ']; %#ok<AGROW>
%		case 'velocity'
%			format_string = [format_string '%07.3f ']; %#ok<AGROW>
		case 'velocity kmh'
			format_string = [format_string '%07.3f ']; %#ok<AGROW>
		otherwise
			format_string = [format_string '%+1.6E ']; %#ok<AGROW>
		end
	end
	
	format_string = [format_string '\n'];
	
	% recalculate the HHMMSS.ss literal times from seconds
	if (time_channel > 0)
		% preallocate (and discard old literals)
		vbo.channels(time_channel).literal_data = vbo.channels(time_channel).data;
		
		for cell = 1:size(vbo.channels(time_channel).data,1)
			sec = vbo.channels(time_channel).data(cell);
			hour = floor(sec / 3600);
			sec = sec - hour * 3600;
			min = floor(sec / 60);
			sec = sec - min * 60;
			vbo.channels(time_channel).literal_data(cell) = ...
				(10000 * hour) + (100 * min) + sec;
		end
	end
	
	comma_sep = 0;
	if (isfield(vbo, 'decimal_separator'))
		comma_sep = 1;										% (any special separator)
	end
	
	% output data to file
	for cell = 1:size(vbo.channels(1).data,1)
		line = zeros(1, size(vbo.channels,2));				% preallocate
		
		for channel_num = 1:size(vbo.channels,2)
			if (isempty(vbo.channels(channel_num).literal_data))
				% just use plain data (most channels)
				line(channel_num) = vbo.channels(channel_num).data(cell);
			else
				% processed data available (time channel)
				line(channel_num) = vbo.channels(channel_num).literal_data(cell);
			end
		end
	
		if (comma_sep)
			[BUF, err] = sprintf(format_string, line);
			BUF = strrep(BUF, '.', vbo.decimal_separator);	% local decimal separator
			fprintf(fid, BUF);
		else
			fprintf(fid, format_string, line);				% print directly
		end
	end
	
end

%% write comments
%  append "Saved from MATLAB [datestamp]" on the fly rather than into
%  comments - so it doesn't grow if we save repeatedly (only on load-save)
function write_comments
	num_lines = size(vbo.sections(secnum).content,2);
	
	for line_num = 1:(num_lines - 1)
		fprintf(fid, '%s\n', char(vbo.sections(secnum).content(line_num)));
	end
	
	fprintf(fid, 'Saved from MATLAB on %s\n', savetime);
	
	% print last comment if it's not empty (it probably is)
	if (1 ~= strcmp('', vbo.sections(secnum).content(num_lines)))
		fprintf(fid, '%s\n\n', char(vbo.sections(secnum).content(num_lines)));
	else
		fprintf(fid, '\n');
	end
end

%% write other unprocessed sections
function write_misc
	for line = vbo.sections(secnum).content
		fprintf(fid, '%s\n', char(line));
	end
end

%% end main function
end		% of main write function