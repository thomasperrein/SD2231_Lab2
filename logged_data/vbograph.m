% vbograph - demo graphing a loaded VBO file, time vs velocity

xvar = 'time';
yvar = 'slip_cog';

%% have we loaded a vbo?

global vbo;
if (0 == size(vbo,1))
	error 'Load a VBO file first.';
end

%% find the channels by name

xchan = 0;
ychan = 0;

for chan = 1:size(vbo.channels,2)
	if (1 == strcmp(xvar, vbo.channels(chan).name))
		xchan = chan;
	elseif (1 == strcmp(yvar, vbo.channels(chan).name))
		ychan = chan;
	end
end

if (xchan == 0)
	fprintf(1, '\n*** X-channel "%s" not found in VBO file!\n\n', xvar);
	error('See above');
end
if (ychan == 0)
	fprintf(1, '\n*** Y-channel "%s" not found in VBO file!\n\n', yvar);
	error('See above');
end

plot(vbo.channels(xchan).data, vbo.channels(ychan).data);

xlabel(sprintf('%s (%s)', vbo.channels(xchan).name, vbo.channels(xchan).units));
ylabel(sprintf('%s (%s)', vbo.channels(ychan).name, vbo.channels(ychan).units));
