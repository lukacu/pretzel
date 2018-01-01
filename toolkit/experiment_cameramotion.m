function [files, metadata] = experiment_cameramotion(tracker, sequence, directory, parameters, scan)

	files = {};
	metadata.completed = true;
	cache = get_global_variable('experiment_cache', true);
	silent = get_global_variable('experiment_silent', false);

	defaults = struct('repetitions', 15, 'skip_tags', {{}}, 'skip_initialize', 0, 'failure_overlap', 0);
	context = struct_merge(parameters, defaults);
	metadata.deterministic = false;

	if context.failure_overlap < 0
		error('Illegal failure overlap');
	end;

	time_file = fullfile(directory, sprintf('%s_time.txt', sequence.name));

	times = zeros(sequence.length, context.repetitions);

	if ~scan && cache && exist(time_file, 'file')
		times = csvread(time_file);
	end;

	r = context.repetitions;

	if isfield(tracker, 'metadata') && isfield(tracker.metadata, 'deterministic') && tracker.metadata.deterministic
		r = 1;
	end

	check_deterministic = ~(scan && nargout < 2); % Ensure faster execution when we only want a list of files by ommiting determinisim check.

	for i = 1:r

		result_file = fullfile(directory, sprintf('%s_%03d.txt', sequence.name, i));

		if cache && exist(result_file, 'file')
		    files{end+1} = result_file; %#ok<AGROW>
		    continue;
		end;

		if check_deterministic && i == 4 && is_deterministic(sequence, 3, directory)
		    if ~silent
		        print_debug('Detected a deterministic tracker, skipping remaining trials.');
		    end;
		    metadata.deterministic = true;
		    break;
		end;

		if scan
		    metadata.completed = false;
		    continue;
		end;

		print_indent(1);

		print_text('Repetition %d', i);

		context.repetition = i;

		data.context = context;

		[trajectory, elapsed] = run_simulator(tracker, sequence, context);

		times(:, i) = elapsed;
		write_trajectory(result_file, trajectory);
		csvwrite(time_file, times);

		print_indent(-1);
	end;

	if exist(time_file, 'file')
		files{end+1} = time_file;
	else
		metadata.completed = false;
	end;

end

function [trajectory, elapsed] = run_simulator(tracker, sequence, context)

	pretzel_executable = get_global_variable('pretzel_simulator', '');

	if isempty(pretzel_executable)
		error('PreTZel support not available (client binary not found)');
	end;

	defaults = struct('directory', tempname, 'skip_labels', {{}}, 'skip_initialize', 1, 'failure_overlap', -1);

	context = struct_merge(context, defaults);

    mkpath(context.directory);
    
	groundtruth_file = fullfile(sequence.directory, sequence.file);
	video_file = fullfile(sequence.directory, sequence.video);

	% Generate an initialization region file

	output_file = fullfile(context.directory, 'output.txt');
	timing_file = fullfile(context.directory, 'timing.txt');

	debug = get_global_variable('trax_debug', false);

	arguments = '';

	if debug
		arguments = [arguments, ' -d'];
	end;

	if (context.failure_overlap >= 0)
		arguments = [arguments, sprintf(' -f %.5f', context.failure_overlap)];
	end;

	if (context.skip_initialize > 0)
		arguments = [arguments, sprintf(' -r %d', context.skip_initialize)];
	end;

	if ispc
		library_var = 'PATH';
	else
		library_var = 'LD_LIBRARY_PATH';
	end;

	% Make Matlab use system libraries
	if ~isempty(tracker.linkpath)
		userpath = tracker.linkpath{end};
		if length(tracker.linkpath) > 1
		    userpath = [sprintf(['%s', pathsep], tracker.linkpath{1:end-1}), userpath];
		end;
		library_paths = {userpath, strsplit(getenv(library_var), pathsep)};
	else
		library_paths = strsplit(getenv(library_var), pathsep);
	end;

	environment = {'TRAX=1', sprintf('%s=%s', library_var, strjoin(library_paths, pathsep))};

	if ~isempty(tracker.environment)
		environment = cat(1, tracker.environment(:), environment(:));
	end;

	% Specify timeout period
	timeout = get_global_variable('trax_timeout', 30);
	arguments = [arguments, sprintf(' -t %d', timeout)];

	if ~isempty(tracker.parameters) && iscell(tracker.parameters)
		for i = 1:size(tracker.parameters, 1)
		    arguments = [arguments, sprintf(' -p "%s=%s"', ...
		        tracker.trax_parameters{i, 1}, num2str(tracker.parameters{i, 2}))]; %#ok<AGROW>
		end
	end

	% Hint to tracker that it should use trax
	arguments = [arguments, strjoin(cellfun(@(x) sprintf(' -e "%s" ', x), environment, 'UniformOutput', false), ' ')];

    % If we are running Matlab tracker on Windows, we have to use TCP/IP
    % sockets
    if ispc && strcmpi(tracker.interpreter, 'matlab')
        arguments = [arguments, ' -X'];
    end

	arguments = [arguments, ' -c ', sequence.pretzel_camera_view];

	if ispc
	command = sprintf('"%s" %s -V "%s" -G "%s" -O "%s" -T "%s" -- %s', pretzel_executable, ...
		arguments, video_file, groundtruth_file, output_file, ...
		timing_file, tracker.command);
	else
	command = sprintf('%s %s -V "%s" -G "%s" -O "%s" -T "%s" -- %s', pretzel_executable, ...
		arguments, video_file, groundtruth_file, output_file, ...
		timing_file, tracker.command);
	end

	error_message = [];

    log_directory = fullfile(get_global_variable('directory'), 'logs', tracker.identifier);
    mkpath(log_directory);
    
	[status, output, elapsed] = external(command, 'Directory', context.directory);

	if status ~= 0
		print_debug('WARNING: System command has not exited normally.');
		if ~isempty(output)
            timestamp = datestr(now, 30);
		    print_debug('Writing client output to a log file.');
		    fid = fopen(fullfile(log_directory, [timestamp, '.log']), 'w');
		    fprintf(fid, '%s', output);
		    fclose(fid);
		end;
		error_message = sprintf('Error during tracker execution. Report written to "%s"', log_directory);
	else
		try
			trajectory = read_trajectory(output_file);
			elapsed = elapsed / sequence.length;
		catch
			error_message = 'Error reading tracker result.';
		end;
	end;

	if isempty(error_message)

		if get_global_variable('cleanup', 1)
			try
				% clean-up temporary directory
				delpath(context.directory);
			catch
				print_debug('WARNING: unable to remove directory %s', context.directory);
			end
		end;

	else
		error(error_message);
	end

end
