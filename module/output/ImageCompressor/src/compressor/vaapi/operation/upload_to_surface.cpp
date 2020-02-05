#include "upload_to_surface.h"

#include <fmt/format.h>

#include "../vaapi_error_category.hpp"
#include "utility/vision/fourcc.h"

namespace module::output::compressor::vaapi::operation {

/**
 * An std::copy like function that takes an rgb array and an rgba array and unpacks the rgb values into
 * four bytes
 */
void unpack_copy(const uint8_t* const& rgb_begin, const uint8_t* const& rgb_end, uint8_t* rgba_ptr) {

    // Cast rgba into a uint32_t
    uint32_t* rgba = reinterpret_cast<uint32_t*>(rgba_ptr);

    // Move through 3 bytes at a time, but skip the last element (last 3 bytes) as it is not large
    // enough to do a 4 byte copy without accessing out of bounds memory
    const uint8_t* rgb;  // Needed after
    for (rgb = rgb_begin; rgb < rgb_end - 3; rgb += 3) {
        *(rgba++) = *reinterpret_cast<const uint32_t*>(rgb) | 0xFF000000;
    }

    // The rgb array is one byte shorter than the rgba array, so we have to copy the last byte by hand
    uint8_t* rgba_last = reinterpret_cast<uint8_t*>(rgba);
    rgba_last[0]       = rgb[0];
    rgba_last[1]       = rgb[1];
    rgba_last[2]       = rgb[2];
    rgba_last[3]       = 0xFF;
}

void image_to_buffer(uint8_t* buffer,
                     const VAImage& surface_image,
                     const std::vector<uint8_t>& data,
                     const uint32_t& width,
                     const uint32_t& height,
                     const uint32_t& format) {
    int channels = 1;
    switch (format) {
        case utility::vision::fourcc("BGRA"):
        case utility::vision::fourcc("RGBA"): channels = 4; [[fallthrough]];
        case utility::vision::fourcc("BGGR"):
        case utility::vision::fourcc("RGGB"):
        case utility::vision::fourcc("GRBG"):
        case utility::vision::fourcc("GBRG"):
        case utility::vision::fourcc("PBG8"):
        case utility::vision::fourcc("PRG8"):
        case utility::vision::fourcc("PGR8"):
        case utility::vision::fourcc("PGB8"):
        case utility::vision::fourcc("Y8  "):
        case utility::vision::fourcc("GRAY"):
        case utility::vision::fourcc("GREY"): {
            // The simple case, the pitch matches width so we can just copy
            if (surface_image.pitches[0] == width * channels) {
                std::copy(data.begin(), data.end(), buffer);
            }
            // Tricky case, we have to copy the image row by row
            else {
                for (uint32_t i = 0; i < height; ++i) {
                    std::copy(std::next(data.begin(), i * width * channels),
                              std::next(data.begin(), (i + 1) * width * channels),
                              buffer + surface_image.pitches[0] * i);
                }
            }
        } break;
        case utility::vision::fourcc("Y16 "): {
            // 16 bit greyscale needs to be downcast
            // The simple case, the pitch matches width so we can just copy
            if (surface_image.pitches[0] == width * channels) {
                for (uint32_t i = 0; i < width * height; ++i) {
                    buffer[i] = data[i * 2 + 1];
                }
            }
            // Tricky case, we have to copy the image row by row
            else {
                for (uint32_t y = 0; y < height; ++y) {
                    for (uint32_t x = 0; x < width; ++x) {
                        buffer[surface_image.pitches[0] * y + x] = data[(y * width + x) * 2 + 1];
                    }
                }
            }
        } break;
        case utility::vision::fourcc("BGR8"):
        case utility::vision::fourcc("RGB8"):
        case utility::vision::fourcc("RGB3"): {
            channels = 3;
            // The simple case, the pitch matches width so we can just copy
            if (surface_image.pitches[0] == width) {
                unpack_copy(data.data(), data.data() + data.size(), buffer);
            }
            // Tricky case, we have to copy the image row by row
            else {
                for (uint32_t i = 0; i < height; ++i) {
                    unpack_copy(data.data() + i * width * channels,
                                data.data() + (i + 1) * width * channels,
                                buffer + surface_image.pitches[0] * i);
                }
            }
        } break;
        default:
            throw std::runtime_error(
                fmt::format("VAAPI Compressor does not support format {}", utility::vision::fourcc(format)));
    }
}


void upload_to_surface(VADisplay dpy,
                       const std::vector<uint8_t>& data,
                       const uint32_t& width,
                       const uint32_t& height,
                       const uint32_t& format,
                       VASurfaceID surface_id) {

    VAStatus va_status;

    // Get access to the memory on the device
    VAImage surface_image;
    void* ptr = nullptr;
    va_status = vaDeriveImage(dpy, surface_id, &surface_image);
    if (va_status != VA_STATUS_SUCCESS) {
        throw std::system_error(va_status, vaapi_error_category(), "Error while deriving an image from the surface");
    }
    va_status = vaMapBuffer(dpy, surface_image.buf, &ptr);
    if (va_status != VA_STATUS_SUCCESS) {
        throw std::system_error(
            va_status, vaapi_error_category(), "Error when mapping the image's buffer into user space");
    }

    // Copy the image over to the GPU buffer
    image_to_buffer(reinterpret_cast<uint8_t*>(ptr), surface_image, data, width, height, format);

    // Cleanup the buffer and unmap so that VAAPI knows it can use the data now
    va_status = vaUnmapBuffer(dpy, surface_image.buf);
    if (va_status != VA_STATUS_SUCCESS) {
        throw std::system_error(
            va_status, vaapi_error_category(), "Error when unmapping the buffer back to device space");
    }
    va_status = vaDestroyImage(dpy, surface_image.image_id);
    if (va_status != VA_STATUS_SUCCESS) {
        throw std::system_error(va_status, vaapi_error_category(), "Error when destroying the image");
    }
}

}  // namespace module::output::compressor::vaapi::operation
