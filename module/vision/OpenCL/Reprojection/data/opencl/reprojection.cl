// Need CL_DEVICE_IMAGE_SUPPORT
// Compile flags:

constant const uint FORMAT_GREY = 0x59455247;
constant const uint FORMAT_Y12  = 0x20323159;
constant const uint FORMAT_Y16  = 0x20363159;
constant const uint FORMAT_Y411 = 0x31313459;
constant const uint FORMAT_UYVY = 0x59565955;
constant const uint FORMAT_YUYV = 0x56595559;
constant const uint FORMAT_YM24 = 0x34324d59;
constant const uint FORMAT_RGB3 = 0x33424752;
constant const uint FORMAT_JPEG = 0x4745504a;

// bayer formats
constant const uint FORMAT_GRBG = 0x47425247;
constant const uint FORMAT_RGGB = 0x42474752;
constant const uint FORMAT_GBRG = 0x47524247;
constant const uint FORMAT_BGGR = 0x52474742;
constant const uint FORMAT_GR12 = 0x32315247;
constant const uint FORMAT_RG12 = 0x32314752;
constant const uint FORMAT_GB12 = 0x32314247;
constant const uint FORMAT_BG12 = 0x32314742;
constant const uint FORMAT_GR16 = 0x36315247;
constant const uint FORMAT_RG16 = 0x36314752;
constant const uint FORMAT_GB16 = 0x36314247;
constant const uint FORMAT_BG16 = 0x36314742;

/**
 * A function for converting a YCbCr colour to RGBA
 * Based from http://en.wikipedia.org/wiki/YCbCr#JPEG_conversion
 *
 * @param {float4} ycbcr A 4-component YCbCr array (includes alpha for convenience)
 * @returns {float4} A converted RGBA colour (alpha untouched)
 */
float4 YCbCrToRGB(float4 ycbcr) {
    const float factor = 128.0f / 255.0f;

    // conversion numbers have been modified to account for the colour being in the 0-1 range instead of 0-255
    return clamp((float4){ycbcr.x + 1.402f * (ycbcr.z - factor),
                          ycbcr.x - 0.34414f * (ycbcr.y - factor) - 0.71414f * (ycbcr.z - factor),
                          ycbcr.x + 1.772f * (ycbcr.y - factor),
                          ycbcr.w},
                 (float4){0.0f, 0.0f, 0.0f, 0.0f},
                 (float4){1.0f, 1.0f, 1.0f, 1.0f});
}

float fetch(read_only image2d_t raw_image, sampler_t sampler, float2 pos) {
    float colour = (float) (read_imagef(raw_image, sampler, pos).x);
    // printf("pos: (%f, %f), colour: %f\n", pos.x, pos.y, colour);
    return colour;
}

// http://graphics.cs.williams.edu/papers/BayerJGT09/
float4 bayerToRGB(read_only image2d_t raw_image, sampler_t sampler, float2 sample_point, float2 first_red) {
    float4 center = (float4){0.0f, 0.0f, 0.0f, 0.0f};
    center.xy     = sample_point;
    center.zw     = sample_point + first_red;

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

float2 projectCamSpaceToScreen(float3 point, float radians_per_pixel) {
    float theta = acos(point.x);
    if (theta == 0.0f) {
        return (float2){0.0f, 0.0f};
    }

    float r         = theta / radians_per_pixel;
    float sin_theta = sin(theta);
    float px        = r * point.y / (sin_theta);
    float py        = r * point.z / (sin_theta);

    return (float2){px, py};
}

float3 getCamFromScreen(float2 screen, float cam_focal_length_pixels) {
    float3 vec = {cam_focal_length_pixels, screen.x, screen.y};
    return normalize(vec);
}

kernel void debayer(read_only image2d_t input,
                    sampler_t sampler,
                    uint image_format,
                    write_only image2d_t output) {

    const float2 pos        = {(float) (get_global_id(0)), (float) (get_global_id(1))};
    const float4 raw_colour = read_imagef(input, sampler, pos);

    // convert into RGBA colour
    switch (image_format) {
        case FORMAT_YUYV: write_imagef(output, (int2){pos.x, pos.y}, YCbCrToRGB(raw_colour)); break;
        case FORMAT_YM24: write_imagef(output, (int2){pos.x, pos.y}, YCbCrToRGB(raw_colour)); break;
        case FORMAT_JPEG: write_imagef(output, (int2){pos.x, pos.y}, YCbCrToRGB(raw_colour)); break;
        case FORMAT_UYVY: write_imagef(output, (int2){pos.x, pos.y}, YCbCrToRGB(raw_colour)); break;

        case FORMAT_GRBG:
            write_imagef(output, (int2){pos.x, pos.y}, bayerToRGB(input, sampler, pos, (float2){1.0, 0.0}));
            break;

        case FORMAT_RGGB:
            write_imagef(output, (int2){pos.x, pos.y}, bayerToRGB(input, sampler, pos, (float2){0.0, 0.0}));
            break;

        case FORMAT_GBRG:
            write_imagef(output, (int2){pos.x, pos.y}, bayerToRGB(input, sampler, pos, (float2){0.0, 1.0}));
            break;

        case FORMAT_BGGR:
            write_imagef(output, (int2){pos.x, pos.y}, bayerToRGB(input, sampler, pos, (float2){1.0, 1.0}));
            break;

        case FORMAT_RGB3: write_imagef(output, (int2){pos.x, pos.y}, raw_colour); break;
        default: write_imagef(output, (int2){pos.x, pos.y}, raw_colour); break;
    }
}

kernel void projectSphericalToRectilinear(read_only image2d_t input,
                                          sampler_t sampler,
                                          float radians_per_pixel,
                                          float2 input_center,
                                          float cam_focal_length_pixels,
                                          write_only image2d_t output) {

    const float2 pos = {(float) (get_global_id(0)), (float) (get_global_id(1))};

    const float2 output_dimensions = {(float) (get_global_size(0)), (float) (get_global_size(1))};
    const float2 output_center     = (output_dimensions - 1.0f) * 0.5f;
    float2 centered_point = {output_center.x - pos.x, output_center.y - pos.y};

    float2 projected_point =
            projectCamSpaceToScreen(getCamFromScreen(centered_point, cam_focal_length_pixels), radians_per_pixel);

    float2 sample_point = (float2){input_center.x - projected_point.x, input_center.y - projected_point.y};

    write_imagef(output, (int2){pos.x, pos.y}, read_imagef(input, sampler, sample_point));
}
