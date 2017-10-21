kernel void project_radial(global const Scalar4* points,
                           global int* indices,
                           global Scalar4* Rco,
                           const Scalar pixels_per_radian,
                           const int2 dimensions,
                           global int2* out) {

    const int index = get_global_id(0);

    // Get our real index
    const int id = indices[index];

    // Get our LUT node
    Scalar4 ray = points[id];

    // Rotate our ray by our matrix to put it into camera space
    ray = (Scalar4)(dot(Rco[0], ray), dot(Rco[1], ray), dot(Rco[2], ray), 0);

    // Calculate some intermediates
    const Scalar theta     = acos(ray.x);
    const Scalar r         = theta * pixels_per_radian;
    const Scalar sin_theta = sin(theta);

    // Work out our pixel coordinates as a 0 centred image with x to the left and y up (screen space)
    const Scalar2 screen = (Scalar2)(r * ray.y / sin_theta, r * ray.z / sin_theta);

    // Apply our offset to move into image space (0 at top left, x to the right, y down)
    const Scalar2 image = (Scalar2)((Scalar)(dimensions.x - 1) * 0.5, (Scalar)(dimensions.y - 1) * 0.5) - screen;

    // Apply our lens centre offset
    // TODO apply this

    // Store our output coordinates
    out[index] = (int2)(round(image.x), round(image.y));
}
