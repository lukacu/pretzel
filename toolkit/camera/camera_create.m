function [ camera ] = camera_create( width, height, dpi )

camera = struct('width', width, 'height', height, 'dpi', dpi, 'f', 1);
camera.rotation = [0, 0, 0];

end

