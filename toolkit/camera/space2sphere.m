function [ res ] = space2sphere( src )

phi = atan2(src(:, 2), src(:, 1));
theta = acos(src(:, 3) ./ sqrt(src(:, 1) .* src(:, 1) + src(:, 2) .* src(:, 2) + src(:, 3) .* src(:, 3)));
res = [phi, theta];

end

