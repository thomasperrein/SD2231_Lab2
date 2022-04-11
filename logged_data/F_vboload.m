%
%	Racelogic VBO file loader
%
function F_vboload
global vbo vbox_file_name;

%% cls
%home;

%% open file
filename=vbox_file_name;

[fid, message] = fopen(filename, 'rt' );

if (-1 == fid)
	error( message );
else
	%fprintf(1, 'Loading %s...\n', filename);
end

%% setup vars
vbo.sections = [];
num_standard_channels = 0;

standard_channels = [];
run vbo_standard_channels

% set bogus section name "created" (will need special handling on save)
section_name = 'created';

%% main read loop
while (~feof(fid))
	buffer = {};
	
	while (~feof(fid))		% read this section
		line = fgetl(fid);
		if ( (size(line,2) > 2) && ('[' == line(1)) && (']' == line(size(line,2))) )
			next_section = line(2:(size(line,2) - 1));
			break;
		else
			buffer = { buffer{:} line };
		end
	end
	
	switch(section_name)
	case 'header'
		handle_header;
	case 'channel units'
		handle_channel_units;
	otherwise
		handle_misc;
	end
	
	if (1 == strcmp('data', next_section))
		handle_data_next;
		% should now be EOF and exit cleanly
	else
		section_name = next_section;
	end
end

fclose(fid);

%disp('Done.');

%% end main function. start nested functions.

%% handle comments and other sections - just store them
function handle_misc
	clear newsec;
	
	newsec.name = section_name;
	newsec.content = buffer;
	vbo.sections = [vbo.sections newsec];
end

%% header section
%%
function handle_header
	clear newsec;
	
	newsec.name = 'header';							% dummy section name
	newsec.content = '(DUMMY)';
	vbo.sections = [vbo.sections newsec];
	
	% last line is probably blank so remove it.
	if (1 == strcmp('', buffer(size(buffer,2))))
		buffer(size(buffer,2)) = [];
	end

%% get channels

	vbo.channels = [];
	
	%fprintf(1, 'Channel names:\n');
	for chan_name = buffer
		clear newchan;
		
		newchan.name = lower(char(chan_name));
		vbo.channels = [vbo.channels newchan];
		
		fprintf(1, ' Channel %-2d = %s\n', size(vbo.channels,2), newchan.name);
	end

%% search for standard channels

	for channel_num = 1:size(vbo.channels,2)
		for std_num = 1:size(standard_channels,1)
			if (1 == strcmp( ...
					standard_channels(std_num,1), ...
					vbo.channels(channel_num).name ...
				))
				num_standard_channels = num_standard_channels + 1;
				vbo.channels(channel_num).units = char(standard_channels(std_num,2));
				break;
			end
		end
	end
	
end

%% channel units section
%%
function handle_channel_units
	clear newsec;
	
	newsec.name = 'channel units';					% dummy section name
	newsec.content = '(DUMMY)';
	vbo.sections = [vbo.sections newsec];

%% apply units to non-standard channels
	unit_num = 1;
	
	for channel_num = 1+num_standard_channels:size(vbo.channels,2)
		if (unit_num > size(buffer, 2))
			break;									% not enough custom units defined
		end
		vbo.channels(channel_num).units = char(buffer(unit_num));
		unit_num = unit_num + 1;
	end

end

%% data section (not buffered first)
%%
function handle_data_next
	clear newsec;
	
	newsec.name = 'data';							% dummy section name
	newsec.content = '(DUMMY)';
	vbo.sections = [vbo.sections newsec];
	
	num_channels = size(vbo.channels, 2);
	
	comma_sep = 0;

	line = fgetl(fid);								% get first line
	
	if (isempty(regexp(line, '\d,\d')))				% comma-separated decimal?
		comma_sep = 0;
	else
		comma_sep = 1;
		vbo.decimal_separator = ',';
	end
	
	for channel_num = 1:num_channels
		vbo.channels(channel_num).data = [];		% create empty structure
	end
	
	while (ischar(line))
		if (comma_sep)
			line = strrep(line, ',', '.');			% replace commas with dots
		end

		A = strread(line, '%f');					% parse line into doubles
		
		if (num_channels ~= size(A, 1))				% invalid data line, bad count
			if (1 < size(line,2))					% ignore empty lines
				fprintf(1, ['Warning: could not interpret data line "%s"\n' ...
					'near byte %d of input file\n'], line, ftell(fid) ...
				);
			end
		else
			for channel_num = 1:num_channels		% append data to channels
				vbo.channels(channel_num).data = ...
					vertcat(vbo.channels(channel_num).data, A(channel_num));
			end
		end	
		line = fgetl(fid);							% get next line
	end
	
	% find and process time channel
	for channel_num = 1:num_channels
		if (1 == strcmp('time', vbo.channels(channel_num).name))
			vbo.channels(channel_num).literal_data = ...
				vbo.channels(channel_num).data;

			for cell = 1:size(vbo.channels(channel_num).literal_data)
				% convert string HHMMSS.ss to seconds
				literal = vbo.channels(channel_num).literal_data(cell);
				
				sec = mod(literal,100);
				min = mod(literal - sec, 10000) / 100;
				hour = (literal - (min*100) - sec) / 10000;
				
				vbo.channels(channel_num).data(cell) = hour * 3600 + min * 60 + sec;
			end
		end
	end
end

%% End subfunctions

end