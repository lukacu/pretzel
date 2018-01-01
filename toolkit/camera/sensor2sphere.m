function [ pout ] = sensor2sphere( camera, pin )

    dx = -(pin(:, 1) - camera.width / 2) ./ (camera.dpi * camera.f);
    dy = -(pin(:, 2) - camera.height / 2) ./ (camera.dpi * camera.f);

    % Compute relative spherical coordinates
    phi = atan2(dy, dx);
    theta = acos(1.0 ./ sqrt(1 + dx .* dx + dy .* dy));

    % Conversion to cartesian and rotation
    matrix = euler2matrix(camera.rotation)';
    t = matrix * [sin(theta) .* cos(phi), sin(theta) .* sin(phi), cos(theta)]';

    % Recompute absolute spherical coordinates
    pout = space2sphere(t');

end

