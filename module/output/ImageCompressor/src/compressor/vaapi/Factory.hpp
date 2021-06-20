#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_FACTORY_HPP
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_FACTORY_HPP

#include "../CompressorFactory.hpp"
#include "CompressionContext.hpp"
#include "Compressor.hpp"

namespace module::output::compressor::vaapi {

    class Factory : public CompressorFactory {
    public:
        Factory(const std::string& device, const std::string& driver, const int& quality);
        Factory(const Factory&) = default;
        Factory(Factory&&)      = default;
        Factory& operator=(const Factory&) = default;
        Factory& operator=(Factory&&) = default;
        virtual ~Factory();

        std::shared_ptr<compressor::Compressor> make_compressor(const uint32_t& width,
                                                                const uint32_t& height,
                                                                const uint32_t& format) override;

    private:
        /// The file descriptor we opened for the DRM device
        int fd;
        /// The compression context that we are using, contains information about the compressor and OpenCL contexts
        CompressionContext cctx;
        /// The quality that this compressor is configured for
        int quality;
    };

}  // namespace module::output::compressor::vaapi

#endif  //  MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_FACTORY_HPP
