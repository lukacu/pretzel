function [angles] = look_at(at, up)

    z = at;
    z = z / norm(z);
    x = cross(up, z);
    x = x / norm(x);
    y = cross(z, x);

    rotation = [x; y; z];
    angles = matrix2euler(rotation);

end

