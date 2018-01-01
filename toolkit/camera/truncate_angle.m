function [angle] = truncate_angle(angle)

altz = angle < 0;
angle(altz) = angle(altz) + ceil(-angle(altz) / (2 * pi)) * (2 * pi);
angle(~altz) = angle(~altz) - floor(angle(~altz) / (2 * pi)) * (2 * pi);

end

