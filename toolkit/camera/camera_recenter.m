function [camera] = camera_recenter(camera, region, up)

    mr = mean(sphere2space(region), 1);

    camera.rotation = look_at(mr, up);

    camera = focus_to_region(camera, region);
    
    rect = camera_project(camera, region);

    center = nanmean(rect, 1);
    p = sensor2sphere(camera, center);

    theta = truncate_angle(p(2));
    phi = truncate_angle(p(1));

    if (theta > pi) 
        theta = 2 * pi - theta;
        phi = truncate_angle(phi + pi);
    end;

    at = [sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)];
    camera.rotation = look_at(at, up);
 
end
    
function [camera] = focus_to_region(camera, region)

    matrix = euler2matrix(camera.rotation)';
    
    t = (matrix * sphere2space(region)')';
    
    fbest = ones(size(t, 1), 1) * camera.f;
    
    fbest(t(:, 1) ~= 0) = abs((t(t(:, 1) ~= 0, 3) ./ t(t(:, 1) ~= 0, 1)) * (camera.width / 2) / camera.dpi);
    fbest(t(:, 2) ~= 0) = abs((t(t(:, 2) ~= 0, 3) ./ t(t(:, 2) ~= 0, 2)) * (camera.width / 2) / camera.dpi);
    
    camera.f = min(fbest);

end
