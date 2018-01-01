function [ visible ] = is_visible( camera, region )

	projected = camera_project(camera, region);

    visible = ~all(isnan(projected), 2) & ~(projected(:, 1) < 0 | projected(:, 1) > camera.width | projected(:, 2) < 0 | projected(:, 2) > camera.height);

end

