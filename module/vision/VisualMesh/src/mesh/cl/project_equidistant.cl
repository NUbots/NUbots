/**
 * Projects visual mesh points to a Fisheye camera with equidistant projection
 *
 * @param points        VisualMesh unit vectors
 * @param indices       map from local indices to global indices
 * @param Rco           rotation from the observation space to camera space
 *                      note that while this is a 4x4, that is for alignment, no translation should exist
 * @param f             the focal length of the lens measured in pixels
 * @param dimensions    the dimensions of the input image
 * @param out           the output image coordinates
 */
kernel void project_equidistant(global const Scalar4* points,
                                global int* indices,
                                const Scalar16 Rco,
                                const Scalar f,
                                const int2 dimensions,
                                global Scalar2* out) {

    const int index = get_global_id(0);

    // Get our real index
    const int id = indices[index];

    // Get our LUT point
    Scalar4 ray = points[id];

    // Rotate our ray by our matrix to put it into camera space
    ray = (Scalar4)(dot(Rco.s0123, ray), dot(Rco.s4567, ray), dot(Rco.s89ab, ray), 0);

    // Calculate some intermediates
    const Scalar theta     = acos(ray.x);
    const Scalar r         = f * theta;
    const Scalar sin_theta = sin(theta);

    // Work out our pixel coordinates as a 0 centred image with x to the left and y up (screen space)
    const Scalar2 screen = (Scalar2)(r * ray.y / sin_theta, r * ray.z / sin_theta);

    // Apply our offset to move into image space (0 at top left, x to the right, y down)
    const Scalar2 image =
        (Scalar2)((Scalar)(dimensions.x - 1) * (Scalar)(0.5), (Scalar)(dimensions.y - 1) * (Scalar)(0.5)) - screen;

    // Apply our lens centre offset
    // TODO apply this

    // Store our output coordinates
    out[index] = image;
}
