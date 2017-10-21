kernel void project_equirectangular(global const Scalar4* points,
                                    global const int* indices,
                                    global const Scalar4* Rco,  // This is only Scalar4 for alignment concerns
                                    const Scalar focal_length_pixels,
                                    const int2 dimensions,
                                    global int2* out) {

    const int index = get_global_id(0);

    // Get our real index
    const int id = indices[index];

    // Get our LUT node
    Scalar4 ray = points[id];

    // Rotate our ray by our matrix to put it in the camera space
    ray = (Scalar4)(dot(Rco[0], ray), dot(Rco[1], ray), dot(Rco[2], ray), 0);

    // Work out our pixel coordinates as a 0 centred image with x to the left and y up (screen space)
    const Scalar2 screen = (Scalar2)(focal_length_pixels * ray.y / ray.x, focal_length_pixels * ray.z / ray.x);

    // Apply our offset to move into image space (0 at top left, x to the right, y down)
    const Scalar2 image = (Scalar2)((Scalar)(dimensions.x - 1) * 0.5, (Scalar)(dimensions.y - 1) * 0.5) - screen;

    // Store our output coordinates
    out[index] = (int2)(round(image.x), round(image.y));
}
