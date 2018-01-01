function [ camera ] = camera_rescale( camera, region, relative_size )

	projected = camera_project(camera, region);
    

    a = 1:size(projected, 1);
    b = circshift(a, [0, 1]);
    area = (projected(b, 1) + projected(a, 1)) .* (projected(b, 2) - projected(a, 2));
    area = abs(sum(area)) * 0.5;
    
	new_f = camera.f * sqrt((camera.width * camera.height * relative_size) / (area));
	camera.f = new_f;

end

