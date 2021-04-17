function conversion = sphericalToCartesian(inputVec)
%this function converts spherical coordinate vectors (r, theta, phi) to
%cartesian coordinate vectors (x, y, z)
distance = double(inputVec(1));
cos_theta = double(cos(inputVec(2)));
sin_theta = double(sin(inputVec(2)));
cos_phi = double(cos(inputVec(3)));
sin_phi = double(sin(inputVec(3)));
x = double(distance * cos_theta * sin_phi);
y = double(distance * sin_theta * sin_phi);
z = double(distance * cos_phi);
conversion = [x y z];