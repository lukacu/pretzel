function [o] = sphere2space(p)

    theta = truncate_angle(p(:, 2));
    phi = truncate_angle(p(:, 1));

    if (theta > pi) 
        theta = 2 * pi - theta;
        phi = truncate_angle(phi + pi);
    end;

    o = [sin(theta) .* cos(phi), sin(theta) .* sin(phi), cos(theta)];

end

