function [ pout ] = camera_unproject( camera, pin )

    pout = sensor2sphere(camera, pin);

end

