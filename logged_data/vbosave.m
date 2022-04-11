% Racelogic VBO save wrapper.
% - Saves the vbo structure including any new channels.
% - Converts the 'data' member of the time channel (seconds)
%   to 'literal_data'(HHMMSS.ss).
% - If 'literal_data' is populated for any other channel it will be
%	saved instead of 'data'.

global vbo; %#ok<NUSED>

F_vbosave;