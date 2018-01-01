function [ camera ] = camera_move( camera, offset )

	% Move camera to look at a relative offset vector in its image space.

	relative = [camera.width / 2 + offset(1), camera.height / 2 + offset(2)];
    
	% Calcuate the spherical coordinates of the offset vector for zero origin
    tcamera = camera; tcamera.rotation = [0, 0, 0];
	vector = sensor2sphere(tcamera, relative);

	% Obtain existing camera rotation matrix and pre-multiply it with a move matrix.
	camera_matrix = euler2matrix(camera.rotation);

    
    theta = truncate_angle(vector(2));
    phi = truncate_angle(vector(1));

    if (theta > pi) 
        theta = 2 * pi - theta;
        phi = truncate_angle(phi + pi);
    end;

    at = [sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)];

	move_matrix = euler2matrix(look_at(at, camera_matrix * [0, 1, 0]'));

	camera.rotation = matrix2euler(move_matrix * camera_matrix);

end

