/**
 * Project visual mesh points to a rectilinear camera
 *
 * @param points        VisualMesh unit vectors
 * @param indices       map from local indices to global indices
 * @param Rco           rotation from the observation space to camera space
 *                      note that while this is a 4x4, that is for alignment, no translation should exist
 * @param f             the focal length of the lens measured in pixels
 * @param dimensions    the dimensions of the input image
 * @param out           the output image coordinates
 */
kernel void project_rectilinear(global const Scalar4* points,
                                global const int* indices,
                                const Scalar16 Rco,
                                const Scalar f,
                                const int2 dimensions,
                                global Scalar2* out) {

    const int index = get_global_id(0);

    // Get our global index
    const int id = indices[index];

    // Get our mesh vector from the LUT
    Scalar4 ray = points[id];

    // Rotate our ray by our matrix to put it in the camera space
    ray = (Scalar4)(dot(Rco.s0123, ray), dot(Rco.s4567, ray), dot(Rco.s89ab, ray), 0);

    // Work out our pixel coordinates as a 0 centred image with x to the left and y up (screen space)
    const Scalar2 screen = (Scalar2)(f * ray.y / ray.x, f * ray.z / ray.x);

    // Apply our offset to move into image space (0 at top left, x to the right, y down)
    const Scalar2 image =
        (Scalar2)((Scalar)(dimensions.x - 1) * (Scalar)(0.5), (Scalar)(dimensions.y - 1) * (Scalar)(0.5)) - screen;

    // Store our output coordinates
    out[index] = image;
}
