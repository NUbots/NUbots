#include "Factory.h"

#include <fcntl.h>
#include <unistd.h>
#include <va/va.h>
#include <va/va_drm.h>
#include <va/va_enc_jpeg.h>

#include <cstring>
#include <nuclear>
#include <system_error>

#include "cl/opencl_context_for_display.h"
#include "cl/opencl_error_category.h"
#include "operation/create_surface.h"
#include "utility/vision/fourcc.h"
#include "vaapi_error_category.hpp"

namespace module::output::compressor::vaapi {

Factory::Factory(const std::string& device, const std::string& driver, const int& quality) : fd(-1), quality(quality) {
    VAStatus va_status;

    // Open the render device
    fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        throw std::system_error(errno, std::system_category(), "Error when opening the GPU rendering device");
    }

    // Get the VA display object from DRM
    cctx.va.dpy = vaGetDisplayDRM(fd);
    if (cctx.va.dpy == nullptr) {
        throw std::system_error(errno, std::system_category(), "Error while connecting to DRM");
    }

    // Setup the error and info callbacks
    vaSetInfoCallback(
        cctx.va.dpy,
        [](void*, const char* message) {
            NUClear::log<NUClear::INFO>(std::string(message, message + std::strlen(message) - 1));
        },
        this);
    vaSetErrorCallback(
        cctx.va.dpy,
        [](void*, const char* message) {
            NUClear::log<NUClear::ERROR>(std::string(message, message + std::strlen(message) - 1));
        },
        this);

    // We use the iHD driver
    va_status = vaSetDriverName(cctx.va.dpy, const_cast<char*>(driver.c_str()));
    if (va_status != VA_STATUS_SUCCESS) {
        throw std::system_error(va_status, vaapi_error_category(), "Error while setting the driver to use");
    }

    // Initialise the VAAPI
    int major_ver, minor_ver;
    va_status = vaInitialize(cctx.va.dpy, &major_ver, &minor_ver);
    if (va_status != VA_STATUS_SUCCESS) {
        throw std::system_error(va_status, vaapi_error_category(), "Error while initialising the VAAPI");
    }

    // Query for the entrypoints for the JPEGBaseline profile
    std::vector<VAEntrypoint> entrypoints(vaMaxNumEntrypoints(cctx.va.dpy));
    int num_entrypoints = 0;
    va_status = vaQueryConfigEntrypoints(cctx.va.dpy, VAProfileJPEGBaseline, entrypoints.data(), &num_entrypoints);
    if (va_status != VA_STATUS_SUCCESS) {
        throw std::system_error(va_status, vaapi_error_category(), "Error while querying entrypoints");
    }
    entrypoints.resize(num_entrypoints);
    if (std::count(entrypoints.begin(), entrypoints.end(), VAEntrypointEncPicture) == 0) {
        throw std::runtime_error("There are no JPEG encoding endpoints");
    }

    // Query for the Render Target format supported
    VAConfigAttrib attrib[2];
    attrib[0].type = VAConfigAttribRTFormat;
    attrib[1].type = VAConfigAttribEncJPEG;
    va_status      = vaGetConfigAttributes(cctx.va.dpy, VAProfileJPEGBaseline, VAEntrypointEncPicture, attrib, 2);
    if (va_status != VA_STATUS_SUCCESS) {
        throw std::system_error(va_status, vaapi_error_category(), "Error while querying endpoint attributes");
    }

    // Render Target should include the following for us to be able to use it
    // VA_RT_FORMAT_YUV400 for black/white
    // VA_RT_FORMAT_RGB32 for RGBA
    if (!((attrib[0].value & VA_RT_FORMAT_RGB32) && (attrib[0].value & VA_RT_FORMAT_YUV400))) {
        throw std::runtime_error("The endpoint did not support the needed Render Target formats");
    }

    VAConfigAttribValEncJPEG jpeg_attrib_val;
    jpeg_attrib_val.value = attrib[1].value;

    /* Set JPEG profile attribs */
    jpeg_attrib_val.bits.arithmatic_coding_mode = 0;
    jpeg_attrib_val.bits.progressive_dct_mode   = 0;
    jpeg_attrib_val.bits.non_interleaved_mode   = 1;
    jpeg_attrib_val.bits.differential_mode      = 0;

    attrib[1].value = jpeg_attrib_val.value;

    // Create Config for the profile and entrypoint with RT format attribute
    va_status = vaCreateConfig(cctx.va.dpy, VAProfileJPEGBaseline, VAEntrypointEncPicture, attrib, 2, &cctx.va.config);
    if (va_status != VA_STATUS_SUCCESS) {
        throw std::system_error(va_status, vaapi_error_category(), "Error while configuring an endpoint");
    }

    /**********
     * OPENCL *
     **********/
    cctx.cl = cl::opencl_context_for_display(cctx.va.dpy);
}

Factory::~Factory() {
    if (cctx.va.dpy) {
        vaTerminate(cctx.va.dpy);
    }
    if (fd != -1) {
        ::close(fd);
    }
}

std::shared_ptr<compressor::Compressor> Factory::make_compressor(const uint32_t width,
                                                                 const uint32_t& height,
                                                                 const uint32_t& format) {
    return std::make_shared<vaapi::Compressor>(cctx, width, height, format, quality);
}

}  // namespace module::output::compressor::vaapi
