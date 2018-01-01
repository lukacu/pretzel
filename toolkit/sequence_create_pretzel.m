function [sequence] = sequence_create_pretzel(directory, metadata)

    [~, name] = fileparts(directory);

    sequence = struct('name', name, 'directory', directory, ...
        'file', metadata.groundtruth, 'video', metadata.data);

    sequence.groundtruth = read_trajectory(fullfile(sequence.directory, sequence.file));

    sequence.length = numel(sequence.groundtruth);

end

