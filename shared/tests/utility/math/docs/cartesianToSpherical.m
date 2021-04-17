function conversion = cartesianToSpherical(inputVec)
%This function converts cartesian coordinate vectors (x,y,z) to spherical form(r,theta,phi)
x = double(inputVec(1));
y = double(inputVec(2));
z = double(inputVec(3));

r = sqrt((x * x) + (y * y) + (z * z));
theta = atan2(y, x);

if x == 0
    phi = 0;
else
    phi = asin(z / r);
end

conversion = [r theta phi];
end
