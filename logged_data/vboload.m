% Racelogic VBO load wrapper
%
% Load a VBO file into the global 'vbo' MATLAB structure, format:
%
%	vbo
%		.sections[]			VBO format file sections
%			.name				String section name e.g. 'comments'
%			.content{}			Cell array of strings, section contents
%								(Not used for 'data', 'header' or
%								 'column units' sections)
%
%		.channels[]			One per data channel
%			.name				String channel name e.g. 'latitude'
%			.units				String units e.g. 'kmh' - may be blank.
%			.data[]				Channel data as vector of doubles
%			.literal_data[]		Channel data in VBO file specific format
%
%	Time is represented in the VBO file in HHMMSS.ss format. It is
%	converted to seconds and stored in .data[] on loading. The contents of
%	.data[] are converted back (into .literal_data[]) on saving.

clear global vbo;
global vbo; %#ok<NUSED>

F_vboload;