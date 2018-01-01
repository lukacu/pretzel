function experiments = stack_amp()

    set_global_variable('bundle', 'http://box.vicos.si/lukacu/amp/sequences.zip');
    set_global_variable('pretzel_views_url', 'http://box.vicos.si/lukacu/amp/views.zip');
    
    baseline.name = 'baseline';
    baseline.pretzel.controller = 'stabilize';
    baseline.latex = 'E_b';

    small.name = 'small';
    small.latex = 'E^s_b';
    small.pretzel.controller = 'stabilize';
	small.pretzel.relative_scale = 0.002;

    rotation1.name = 'rotation_center_slow';
    rotation1.pretzel.controller = 'rotation';
    rotation1.pretzel.rotation_frequency = 6;
    rotation1.latex = 'E^s_r';

    rotation2.name = 'rotation_center_fast';
    rotation2.pretzel.controller = 'rotation';
    rotation2.pretzel.rotation_frequency = 1;
    rotation2.latex = 'E^f_r';

    rotation3.name = 'rotation_offset';
    rotation3.pretzel.controller = 'rotation';
    rotation3.pretzel.rotation_offset = 0.8;
    rotation3.pretzel.rotation_frequency = 4;
    rotation3.latex = 'E_d';

    scale1.name = 'scale_slow';
    scale1.pretzel.controller = 'scale';
    scale1.pretzel.scale_base = 0.03;
    scale1.pretzel.scale_amplitude = 0.02;
    scale1.pretzel.scale_frequency = 6;
    scale1.latex = 'E^{s}_s';

    scale2.name = 'scale_fast';
    scale2.pretzel.controller = 'scale';
    scale2.pretzel.scale_base = 0.03;
    scale2.pretzel.scale_amplitude = 0.02;
    scale2.pretzel.scale_frequency = 1;
    scale2.latex = 'E^{f}_s';

    scale3.name = 'scale_wide';
    scale3.pretzel.controller = 'scale';
    scale3.pretzel.scale_base = 0.04;
    scale3.pretzel.scale_amplitude = 0.03;
    scale3.pretzel.scale_frequency = 4;
    scale3.latex = 'E^{w}_s';

    dynamics1.name = 'dynamics_slow';
    dynamics1.pretzel.controller = 'dynamics';
    dynamics1.pretzel.path_type = 'ellipse';
    dynamics1.pretzel.offset = 0.8;
    dynamics1.pretzel.frequency = 6;
    dynamics1.latex = 'E^{s}_m';

    dynamics2.name = 'dynamics_fast';
    dynamics2.pretzel.controller = 'dynamics';
    dynamics2.pretzel.path_type = 'ellipse';
    dynamics2.pretzel.offset = 0.8;
    dynamics2.pretzel.frequency = 1;
    dynamics2.latex = 'E^{f}_m';

    noise1.name = 'noise_small';
    noise1.pretzel.controller = 'noise';
    noise1.pretzel.motion_noise = 5;
    noise1.latex = 'E^{s}_n';

    noise2.name = 'noise_big';
    noise2.pretzel.controller = 'noise';
    noise2.pretzel.motion_noise = 15;
    noise2.latex = 'E^{w}_n';

    % 0.002 = 25px, 0.01 = 50px

    experiments = {baseline, rotation1, rotation2, rotation3, scale1, scale2, scale3, dynamics1, dynamics2, noise1, noise2, small};
    experiments = cellfun(@configure_experiment, experiments, 'UniformOutput', false);

    views_directory = fullfile(get_global_variable('directory'), 'views');
    bundle_url = get_global_variable('pretzel_views_url');

    if ~exist(views_directory, 'dir')
        print_text('Downloading views bundle from "%s" ...', bundle_url);
        bundle = [tempname, '.zip'];
        try
            mkpath(views_directory);
            urlwrite(bundle_url, bundle);
            unzip(bundle, views_directory);
            delete(bundle);
        catch
            print_text('Unable to retrieve sequence bundle from the server. This is either a connection problem or the server is temporary offline.');
            print_text('Please try to download the bundle manually from %s and uncompress it to %s', bundle_url, directory);
            return;
        end;
    end;
   
end

function experiment = configure_experiment(experiment)

    experiment.converter = @(x) pretzel_converter(x, experiment);
    experiment.type = 'cameramotion';
    experiment.labels = {};

    experiment.labels = {};
    experiment.parameters.repetitions = 10;
    experiment.parameters.burnin = 10;
    experiment.parameters.skip_initialize = 5;
    experiment.parameters.failure_overlap = 0.0;

    experiment.pretzel.width = 640;
    experiment.pretzel.height = 480;
    experiment.pretzel.dpi = 1000;

end


function [sequence] = pretzel_converter(sequence, experiment)

    if isfield(sequence, 'pretzel_camera_view')
        % Do not convert already converted sequences
        return;
    end;

    sequence.pretzel_camera_view = fullfile(get_global_variable('directory'), 'views', experiment.name, [sequence.name, '.txt']);

end

