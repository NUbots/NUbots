const sampler_t bayer_sampler  = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP | CLK_FILTER_NEAREST;
const sampler_t interp_sampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP | CLK_FILTER_LINEAR;


enum FOURCC {
    GREY    = 0x59455247,
    Y12     = 0x20323159,
    Y16     = 0x20363159,
    GRBG    = 0x47425247,
    RGGB    = 0x42474752,
    GBRG    = 0x47524247,
    BGGR    = 0x52474742,
    GR12    = 0x32315247,
    RG12    = 0x32314752,
    GB12    = 0x32314247,
    BG12    = 0x32314742,
    GR16    = 0x36315247,
    RG16    = 0x36314752,
    GB16    = 0x36314247,
    BG16    = 0x36314742,
    Y411    = 0x31313459,
    UYVY    = 0x59565955,
    YUYV    = 0x56595559,
    YM24    = 0x34324d59,
    RGB3    = 0x33424752,
    RGBA    = 0x41424752,
    BGR3    = 0x33524742,
    BGRA    = 0x41524742,
    JPEG    = 0x4745504a,
    UNKNOWN = 0
};


float fetch(read_only image2d_t raw_image, sampler_t sampler, float2 pos) {
    return read_imagef(raw_image, sampler, pos).x;
}

// http://graphics.cs.williams.edu/papers/BayerJGT09/
float4 bayerToRGB(read_only image2d_t raw_image, sampler_t sampler, float2 coord, float2 first_red) {
    float4 center = (float4){0.0f, 0.0f, 0.0f, 0.0f};
    center.xy     = coord;
    center.zw     = coord + first_red;

    float4 x_coord = center.x + (float4){-2.0f, -1.0f, 1.0f, 2.0f};
    float4 y_coord = center.y + (float4){-2.0f, -1.0f, 1.0f, 2.0f};

    float C         = fetch(raw_image, sampler, center.xy);  // ( 0, 0)
    const float4 kC = {0.5f, 0.75f, 0.625f, 0.625f};

    // Determine which of four types of pixels we are on.
    float2 alternate = fmod(floor(center.zw), 2.0f);

    float4 Dvec = (float4){fetch(raw_image, sampler, (float2){x_coord.y, y_coord.y}),   // (-1,-1)
                           fetch(raw_image, sampler, (float2){x_coord.y, y_coord.z}),   // (-1, 1)
                           fetch(raw_image, sampler, (float2){x_coord.z, y_coord.y}),   // ( 1,-1)
                           fetch(raw_image, sampler, (float2){x_coord.z, y_coord.z})};  // ( 1, 1)

    float4 PATTERN = (kC.xyz * C).xyzz;

    // Can also be a dot product with (1,1,1,1) on hardware where that is
    // specially optimized.
    // Equivalent to: D = Dvec.x + Dvec.y + Dvec.z + Dvec.w;
    Dvec.xy += Dvec.zw;
    Dvec.x += Dvec.y;

    float4 value = (float4){fetch(raw_image, sampler, (float2){center.x, y_coord.x}),   // ( 0,-2)
                            fetch(raw_image, sampler, (float2){center.x, y_coord.y}),   // ( 0,-1)
                            fetch(raw_image, sampler, (float2){x_coord.x, center.y}),   // (-1, 0)
                            fetch(raw_image, sampler, (float2){x_coord.y, center.y})};  // (-2, 0)

    float4 temp = (float4){fetch(raw_image, sampler, (float2){center.x, y_coord.w}),   // ( 0, 2)
                           fetch(raw_image, sampler, (float2){center.x, y_coord.z}),   // ( 0, 1)
                           fetch(raw_image, sampler, (float2){x_coord.w, center.y}),   // ( 2, 0)
                           fetch(raw_image, sampler, (float2){x_coord.z, center.y})};  // ( 1, 0)

    // Even the simplest compilers should be able to constant-fold these to avoid the division.
    // Note that on scalar processors these constants force computation of some identical products twice.
    const float4 kA = {-0.125f, -0.1875f, 0.0625f, -0.125f};
    const float4 kB = {0.25f, 0.0f, 0.0f, 0.5f};
    const float4 kD = {0.0f, 0.25f, -0.125f, -0.125f};

    // Conserve constant registers and take advantage of free swizzle on load
    const float4 kE = kA.xywz;
    const float4 kF = kB.xywz;

    value += temp;

    // There are five filter patterns (identity, cross, checker,
    // theta, phi).  Precompute the terms from all of them and then
    // use swizzles to assign to color channels.
    //
    // Channel   Matches
    //   x       cross   (e.g., EE G)
    //   y       checker (e.g., EE B)
    //   z       theta   (e.g., EO R)
    //   w       phi     (e.g., EO R)
    const float A = value.x;
    const float B = value.y;
    const float D = Dvec.x;
    const float E = value.z;
    const float F = value.w;

    // Avoid zero elements. On a scalar processor this saves two MADDs and it has no
    // effect on a vector processor.
    PATTERN.yzw += (kD.yz * D).xyy;

    PATTERN += (kA.xyz * A).xyzx + (kE.xyw * E).xyxz;
    PATTERN.xw += kB.xw * B;
    PATTERN.xz += kF.xz * F;

    float4 result;

    if (alternate.y == 0.0f) {
        if (alternate.x == 0.0f) {
            result = (float4){C, PATTERN.xy, 1.0f};
        }

        else {
            result = (float4){PATTERN.z, C, PATTERN.w, 1.0f};
        }
    }

    else {
        if (alternate.x == 0.0f) {
            result = (float4){PATTERN.w, C, PATTERN.z, 1.0f};
        }

        else {
            result = (float4){PATTERN.yx, C, 1.0f};
        }
    }

    return result;
}


float4 read_image(read_only image2d_t image, const enum FOURCC format, const float2 coordinates) {
    switch (format) {
        case GRBG: {
            return bayerToRGB(image, bayer_sampler, coordinates, (float2)(1.0, 0.0));
        }
        case RGGB: {
            return bayerToRGB(image, bayer_sampler, coordinates, (float2)(0.0, 0.0));
        }
        case GBRG: {
            return bayerToRGB(image, bayer_sampler, coordinates, (float2)(0.0, 1.0));
        }
        case BGGR: {
            return bayerToRGB(image, bayer_sampler, coordinates, (float2)(1.0, 1.0));
        }
        case RGB3:
        case BGRA:
        case RGBA: {
            return read_imagef(image, interp_sampler, coordinates);
        }
        default: { return (float4)(0); }
    }
}

kernel void read_image_to_network(read_only image2d_t image,
                                  const enum FOURCC format,
                                  global float2* coordinates,
                                  global float4* network) {

    const int idx = get_global_id(0);

    // Read our pixel coordinate into the image
    network[idx] = read_image(image, format, coordinates[idx]);
}
