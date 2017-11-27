#include "reprojection.cuh"

// Define our texture.
texture<unsigned char, cudaTextureType2D, cudaReadModeNormalizedFloat> input_image;

__constant__ const unsigned int FORMAT_GREY = 0x59455247;
__constant__ const unsigned int FORMAT_Y12  = 0x20323159;
__constant__ const unsigned int FORMAT_Y16  = 0x20363159;
__constant__ const unsigned int FORMAT_Y411 = 0x31313459;
__constant__ const unsigned int FORMAT_UYVY = 0x59565955;
__constant__ const unsigned int FORMAT_YUYV = 0x56595559;
__constant__ const unsigned int FORMAT_YM24 = 0x34324d59;
__constant__ const unsigned int FORMAT_RGB3 = 0x33424752;
__constant__ const unsigned int FORMAT_JPEG = 0x4745504a;

// bayer formats
__constant__ const unsigned int FORMAT_GRBG = 0x47425247;
__constant__ const unsigned int FORMAT_RGGB = 0x42474752;
__constant__ const unsigned int FORMAT_GBRG = 0x47524247;
__constant__ const unsigned int FORMAT_BGGR = 0x52474742;
__constant__ const unsigned int FORMAT_GR12 = 0x32315247;
__constant__ const unsigned int FORMAT_RG12 = 0x32314752;
__constant__ const unsigned int FORMAT_GB12 = 0x32314247;
__constant__ const unsigned int FORMAT_BG12 = 0x32314742;
__constant__ const unsigned int FORMAT_GR16 = 0x36315247;
__constant__ const unsigned int FORMAT_RG16 = 0x36314752;
__constant__ const unsigned int FORMAT_GB16 = 0x36314247;
__constant__ const unsigned int FORMAT_BG16 = 0x36314742;

/**
 * A function for converting a YCbCr colour to RGBA
 * Based from http://en.wikipedia.org/wiki/YCbCr#JPEG_conversion
 *
 * @param {float4} ycbcr A 4-component YCbCr array (includes alpha for convenience)
 * @returns {float4} A converted RGBA colour (alpha untouched)
 */
__device__ float4 YCbCrToRGB(float4 ycbcr) {
    const float factor = 128.0f / 255.0f;

    // conversion numbers have been modified to account for the colour being in the 0-1 range instead of 0-255
    return make_float4(min(max(ycbcr.x + 1.402f * (ycbcr.z - factor), 0.0f), 1.0f),
                       min(max(ycbcr.x - 0.34414f * (ycbcr.y - factor) - 0.71414f * (ycbcr.z - factor), 0.0f), 1.0f),
                       min(max(ycbcr.x + 1.772f * (ycbcr.y - factor), 0.0f), 1.0f),
                       min(max(ycbcr.w, 0.0f), 1.0f));
}

// http://graphics.cs.williams.edu/papers/BayerJGT09/
__device__ float4 bayerToRGB(float2 sample_point, float2 first_red) {
    float4 center = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
    center.x      = sample_point.x;
    center.y      = sample_point.y;
    center.z      = sample_point.x + first_red.x;
    center.w      = sample_point.y + first_red.y;

    float4 x_coord = center.x + make_float4(-2.0f, -1.0f, 1.0f, 2.0f);
    float4 y_coord = center.y + make_float4(-2.0f, -1.0f, 1.0f, 2.0f);

    float C         = tex2D(input_image, center.x, center.y);  // (0, 0)
    const float4 kC = make_float4(0.5f, 0.75f, 0.625f, 0.625f);

    // Determine which of four types of pixels we are on.
    float2 alternate = make_float2(fmod(floor(center.z), 2.0f), fmod(floor(center.w), 2.0f));

    float4 Dvec = make_float4(tex2D(input_image, x_coord.y, y_coord.y),   // (-1, -1)
                              tex2D(input_image, x_coord.y, y_coord.z),   // (-1,  1)
                              tex2D(input_image, x_coord.z, y_coord.y),   // ( 1, -1)
                              tex2D(input_image, x_coord.z, y_coord.z));  // ( 1,  1)

    const float3 kC_temp = make_float3(kC.x * C, kC.y * C, kC.z * C);
    float4 PATTERN       = make_float4(kC_temp.x, kC_temp.y, kC_temp.z, kC_temp.z);

    // Can also be a dot product with (1,1,1,1) on hardware where that is
    // specially optimized.
    // Equivalent to: D = Dvec.x + Dvec.y + Dvec.z + Dvec.w;
    Dvec.x += Dvec.z;
    Dvec.y += Dvec.w;
    Dvec.x += Dvec.y;

    float4 value = make_float4(tex2D(input_image, center.x, y_coord.x),   // ( 0, -2)
                               tex2D(input_image, center.x, y_coord.y),   // ( 0, -1)
                               tex2D(input_image, x_coord.x, center.y),   // (-1,  0)
                               tex2D(input_image, x_coord.y, center.y));  // (-2,  0)

    float4 temp = make_float4(tex2D(input_image, center.x, y_coord.w),   // (0, 2)
                              tex2D(input_image, center.x, y_coord.z),   // (0, 1)
                              tex2D(input_image, x_coord.w, center.y),   // (2, 0)
                              tex2D(input_image, x_coord.z, center.y));  // (1, 0)

    // Even the simplest compilers should be able to constant-fold these to avoid the division.
    // Note that on scalar processors these constants force computation of some identical products twice.
    const float4 kA = make_float4(-0.125f, -0.1875f, 0.0625f, -0.125f);
    const float4 kB = make_float4(0.25f, 0.0f, 0.0f, 0.5f);
    const float4 kD = make_float4(0.0f, 0.25f, -0.125f, -0.125f);

    // Conserve constant registers and take advantage of free swizzle on load
    const float4 kE = make_float4(kA.x, kA.y, kA.w, kA.z);
    const float4 kF = make_float4(kB.x, kB.y, kB.w, kB.z);

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
    const float2 kD_temp = make_float2(kD.y * D, kD.z * D);
    PATTERN.y += kD_temp.x;
    PATTERN.z += kD_temp.y;
    PATTERN.w += kD_temp.y;

    const float3 kA_temp = make_float3(kA.x * A, kA.y * A, kA.z * A);
    const float3 kE_temp = make_float3(kE.x * E, kE.y * E, kE.w * E);
    PATTERN += make_float4(kA_temp.x, kA_temp.y, kA_temp.z, kA_temp.x);
    PATTERN += make_float4(kE_temp.x, kE_temp.y, kE_temp.x, kE_temp.z);
    PATTERN.x += kB.x * B;
    PATTERN.w += kB.w * B;
    PATTERN.x += kF.x * F;
    PATTERN.z += kF.z * F;

    float4 result;

    if (alternate.y == 0.0f) {
        if (alternate.x == 0.0f) {
            result = make_float4(C, PATTERN.x, PATTERN.y, 1.0f);
        }

        else {
            result = make_float4(PATTERN.z, C, PATTERN.w, 1.0f);
        }
    }

    else {
        if (alternate.x == 0.0f) {
            result = make_float4(PATTERN.w, C, PATTERN.z, 1.0f);
        }

        else {
            result = make_float4(PATTERN.y, PATTERN.x, C, 1.0f);
        }
    }

    return result;
}

__device__ float2 projectCamSpaceToScreen(float3 point, float radians_per_pixel) {
    float theta = acosf(point.x);

    if (theta == 0.0f) {
        return make_float2(0.0, 0.0);
    }

    float r         = theta / radians_per_pixel;
    float sin_theta = sinf(theta);
    float px        = r * point.y / (sin_theta);
    float py        = r * point.z / (sin_theta);

    return make_float2(px, py);
}

__device__ float3 getCamFromScreen(float2 screen, float cam_focal_length_pixels) {
    return normalize(make_float3(cam_focal_length_pixels, screen.x, screen.y));
}

__global__ void projectSphericalToRectilinear(unsigned int image_format,
                                              float radians_per_pixel,
                                              uint2 input_dimensions,
                                              uint2 output_dimensions,
                                              float cam_focal_length_pixels,
                                              unsigned char* output) {

    // Calculate input texture coordinates
    const uint2 pos = make_uint2(blockIdx.x * blockDim.x + threadIdx.x, blockIdx.y * blockDim.y + threadIdx.y);
    const float2 output_center = make_float2((output_dimensions.x - 1.0f) * 0.5f, (output_dimensions.y - 1.0) * 0.5);
    const float2 input_center  = make_float2((input_dimensions.x - 1.0f) * 0.5f, (input_dimensions.y - 1.0) * 0.5);

    float2 centered_point = make_float2(output_center.x - pos.x, output_center.y - pos.y);
    float2 projected_point =
        projectCamSpaceToScreen(getCamFromScreen(centered_point, cam_focal_length_pixels), radians_per_pixel);
    float2 sample_point = make_float2(input_center.x - projected_point.x, input_center.y - projected_point.y);
    float4 colour;

    // convert into RGBA colour
    switch (image_format) {
        case FORMAT_GRBG:
            colour = bayerToRGB(sample_point, make_float2(1.0, 0.0));
            output[pos.y * output_dimensions.x * 3 + pos.x * 3 + 0] =
                __float2uint_rz(clamp(255.0f * colour.x + 0.5f, 0.0f, 255.0f));
            output[pos.y * output_dimensions.x * 3 + pos.x * 3 + 1] =
                __float2uint_rz(clamp(255.0f * colour.y + 0.5f, 0.0f, 255.0f));
            output[pos.y * output_dimensions.x * 3 + pos.x * 3 + 2] =
                __float2uint_rz(clamp(255.0f * colour.z + 0.5f, 0.0f, 255.0f));
            break;

        case FORMAT_RGGB:
            colour = bayerToRGB(sample_point, make_float2(0.0, 0.0));
            output[pos.y * output_dimensions.x * 3 + pos.x * 3 + 0] =
                __float2uint_rz(clamp(255.0f * colour.x + 0.5f, 0.0f, 255.0f));
            output[pos.y * output_dimensions.x * 3 + pos.x * 3 + 1] =
                __float2uint_rz(clamp(255.0f * colour.y + 0.5f, 0.0f, 255.0f));
            output[pos.y * output_dimensions.x * 3 + pos.x * 3 + 2] =
                __float2uint_rz(clamp(255.0f * colour.z + 0.5f, 0.0f, 255.0f));
            break;

        case FORMAT_GBRG:
            colour = bayerToRGB(sample_point, make_float2(0.0, 1.0));
            output[pos.y * output_dimensions.x * 3 + pos.x * 3 + 0] =
                __float2uint_rz(clamp(255.0f * colour.x + 0.5f, 0.0f, 255.0f));
            output[pos.y * output_dimensions.x * 3 + pos.x * 3 + 1] =
                __float2uint_rz(clamp(255.0f * colour.y + 0.5f, 0.0f, 255.0f));
            output[pos.y * output_dimensions.x * 3 + pos.x * 3 + 2] =
                __float2uint_rz(clamp(255.0f * colour.z + 0.5f, 0.0f, 255.0f));
            break;

        case FORMAT_BGGR:
            colour = bayerToRGB(sample_point, make_float2(1.0, 1.0));
            output[pos.y * output_dimensions.x * 3 + pos.x * 3 + 0] =
                __float2uint_rz(clamp(255.0f * colour.x + 0.5f, 0.0f, 255.0f));
            output[pos.y * output_dimensions.x * 3 + pos.x * 3 + 1] =
                __float2uint_rz(clamp(255.0f * colour.y + 0.5f, 0.0f, 255.0f));
            output[pos.y * output_dimensions.x * 3 + pos.x * 3 + 2] =
                __float2uint_rz(clamp(255.0f * colour.z + 0.5f, 0.0f, 255.0f));
            break;

        // We don't handle these.
        case FORMAT_GREY:
        case FORMAT_Y12:
        case FORMAT_Y16:
        case FORMAT_Y411:
        case FORMAT_GR12:
        case FORMAT_RG12:
        case FORMAT_GB12:
        case FORMAT_BG12:
        case FORMAT_GR16:
        case FORMAT_RG16:
        case FORMAT_GB16:
        case FORMAT_BG16:
        case FORMAT_YUYV:
        case FORMAT_YM24:
        case FORMAT_JPEG:
        case FORMAT_UYVY:
        case FORMAT_RGB3:
        default:
            output[pos.y * input_dimensions.x * 3 + pos.x * 3 + 0] = 0;
            output[pos.y * input_dimensions.x * 3 + pos.x * 3 + 1] = 0;
            output[pos.y * input_dimensions.x * 3 + pos.x * 3 + 2] = 0;
            break;
    }
}

cudaError_t launchKernel(const unsigned char* input,
                         unsigned int image_format,
                         float radians_per_pixel,
                         uint2 input_dimensions,
                         uint2 output_dimensions,
                         float cam_focal_length_pixels,
                         unsigned char* output) {

    cudaError_t err;

    // Set texture reference parameters
    input_image.addressMode[0] = cudaAddressModeClamp;  // Clamp to edge
    input_image.addressMode[1] = cudaAddressModeClamp;
    input_image.filterMode     = cudaFilterModePoint;  // No interpolation
    input_image.normalized     = false;                // Texture coordinates are not normalised.

    // Allocate device memory for texture.
    unsigned char* texture;
    size_t pitch;
    if ((err = cudaMallocPitch(&texture, &pitch, input_dimensions.x, input_dimensions.y)) != cudaSuccess) {
        return err;
    }

    // Copy texture to device.
    if ((cudaMemcpy2D(
            texture, pitch, input, input_dimensions.x, input_dimensions.x, input_dimensions.y, cudaMemcpyHostToDevice))
        != cudaSuccess) {
        cudaFree(texture);
        return err;
    }

    // Bind texture to device memory.
    size_t offset;
    if ((cudaBindTexture2D(&offset,
                           input_image,
                           texture,
                           cudaCreateChannelDesc<unsigned char>(),
                           input_dimensions.x,
                           input_dimensions.y,
                           pitch))
        != cudaSuccess) {
        cudaFree(texture);
        return err;
    }

    // Allocate memory for kernel output.
    unsigned char* result;
    if ((cudaMalloc(&result, output_dimensions.x * output_dimensions.y * 3 * sizeof(unsigned char))) != cudaSuccess) {
        cudaUnbindTexture(input_image);
        cudaFree(texture);
        return err;
    }

    // Set up kernel execution parameters.
    dim3 dimBlock(16, 16);
    dim3 dimGrid((output_dimensions.x + dimBlock.x - 1) / dimBlock.x,
                 (output_dimensions.y + dimBlock.y - 1) / dimBlock.y);

    // Execute the kernel.
    projectSphericalToRectilinear<<<dimGrid, dimBlock>>>(
        image_format, radians_per_pixel, input_dimensions, output_dimensions, cam_focal_length_pixels, result);

    // Check for any errors.
    if ((err = cudaGetLastError()) != cudaSuccess) {
        cudaUnbindTexture(input_image);
        cudaFree(texture);
        cudaFree(output);
        return err;
    }

    // Copy the result out of the device.
    if ((err = cudaMemcpy(output, result, output_dimensions.x * output_dimensions.y * 3, cudaMemcpyDeviceToHost))
        != cudaSuccess) {
        cudaUnbindTexture(input_image);
        cudaFree(texture);
        cudaFree(output);
        return err;
    }

    // Clean up.
    cudaUnbindTexture(input_image);
    cudaFree(texture);
    cudaFree(result);

    return cudaSuccess;
}
