#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_COMPRESSOR_HPP
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_COMPRESSOR_HPP

#include <array>
#include <memory>
#include <va/va.h>

#include "../Compressor.hpp"
#include "CompressionContext.hpp"
#include "cl/wrapper.hpp"

namespace module::output::compressor::vaapi {

    struct Compressor : public compressor::Compressor {

        Compressor(const CompressionContext& cctx,
                   const uint32_t& width,
                   const uint32_t& height,
                   const uint32_t& format,
                   const int& quality);
        Compressor(const Compressor&)            = default;
        Compressor(Compressor&&)                 = default;
        Compressor& operator=(const Compressor&) = default;
        Compressor& operator=(Compressor&&)      = default;
        virtual ~Compressor();

        std::vector<uint8_t> compress(const std::vector<uint8_t>& data) override;

        // Contexts and buffers used for compressing images
        CompressionContext cctx;
        VASurfaceID surface;
        VAContextID context;
        VABufferID encoded;
        std::array<VABufferID, 6> buffers;

        // OpenCL things
        cl::command_queue command_queue;
        cl::program mosaic_program;
        cl::kernel mosaic_kernel;
        cl::mem cl_surface;
        cl::mem cl_image;

        // Parameters this compressor was created with
        uint32_t width;
        uint32_t height;
        uint32_t format;
    };

}  // namespace module::output::compressor::vaapi

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_COMPRESSOR_HPP
