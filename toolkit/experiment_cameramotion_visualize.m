function experiment_cameramotion_visualize(experiment, tracker, sequence)

    show_groundtruth = get_global_variable('pretzel_show_groundtruth', false);

    player_command = get_global_variable('pretzel_player');
    
    if show_groundtruth
        trajectories = {sprintf('%s=%s', 'groundtruth', fullfile(sequence.directory, 'groundtruth.txt'))};
    else
        trajectories = {};
    end;

    experiment_directory = fullfile(tracker.directory, experiment.name);
    sequence_directory = fullfile(experiment_directory, sequence.name);

    for i = 1:experiment.parameters.repetitions;

        tfile = fullfile(sequence_directory, ...
            sprintf('%s_%03d.txt', sequence.name, i));

        if exist(tfile, 'file')
            trajectories{end+1} = sprintf('%s=%s', tracker.identifier, tfile); %#ok<AGROW>
        end;

    end;

	sequence_views = sequence.pretzel_camera_view;

    video = fullfile(sequence.directory, sequence.video);

	command = sprintf('%s -p -d -0 -c "%s" "%s" %s', player_command, sequence_views, video, strjoin(trajectories, ' '));

	external(command);

end
