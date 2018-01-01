function [angles] = matrix2euler(rotation)

    sy = sqrt(rotation(1, 1) * rotation(1, 1) + rotation(2, 1) * rotation(2, 1));

    if (sy < 10 * eps)

        angles = [atan2(-rotation(2, 3), rotation(2, 2)), ...
                atan2(-rotation(3, 1), sy), 0];
    else 

        angles = [atan2(rotation(3, 2), rotation(3, 3)), ...
            atan2(rotation(3, 1), sy), ...
             atan2(rotation(2, 1), rotation(1, 1))];
    end;

end