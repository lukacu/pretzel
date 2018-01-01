function [ camera ] = camera_rotate(camera, angle )

	% Rotate camera around its forward (z) axis.

	camera_matrix = euler2matrix(camera.rotation);
	rotation_matrix = euler2matrix([0, 0, angle]);
	camera.rotation = matrix2euler(rotation_matrix * camera_matrix);

end

