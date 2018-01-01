function [ pout ] = camera_project( camera, pin )

    space = sphere2space(pin);
    matrix = euler2matrix(camera.rotation);

    t = (matrix * space')';
    
    visible = t(:, 3) > min((camera.f / 3), 0.5);
    
    pout = nan(size(pin));
    
    pout(visible, 1) =  (-camera.f * (t(visible, 1) ./ t(visible, 3)) * camera.dpi + camera.width / 2);
    pout(visible, 2) = (-camera.f * (t(visible, 2) ./ t(visible, 3)) * camera.dpi + camera.height / 2);

end

