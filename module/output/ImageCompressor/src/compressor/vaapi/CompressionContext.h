#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_COMPRESSIONCONTEXT_H
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_COMPRESSIONCONTEXT_H

#include <CL/cl.h>
#include <CL/cl_va_api_media_sharing_intel.h>
#include <va/va.h>

#include "cl/wrapper.h"

namespace module::output::compressor::vaapi {

struct CompressionContext {

    struct VAAPIContext {
        /// A shared pointer to the display object (gets terminated on destruction)
        VADisplay dpy;
        /// The configuration we made for compressing jpegs
        VAConfigID config;
    } va;

    struct OpenCLContext {
        /// The OpenCL device we are executing on
        cl_device_id device;
        /// The OpenCL context we will do operations on
        cl::context context;
        /// The OpenCL extension function that maps surfaces into opencl mem objects
        clCreateFromVA_APIMediaSurfaceINTEL_fn mem_from_surface;
        /// The OpenCL extension function that acquires surfaces from VAAPI
        clEnqueueAcquireVA_APIMediaSurfacesINTEL_fn acquire_surface;
        /// The OpenCL extension function that releases surfaces back to VAAPI
        clEnqueueReleaseVA_APIMediaSurfacesINTEL_fn release_surface;
    } cl;

    /// The quality that this compressor is configured for
    int quality;
};

}  // namespace module::output::compressor::vaapi

#endif  //  MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_COMPRESSIONCONTEXT_H
