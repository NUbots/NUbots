#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_FACTORY_HPP
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_FACTORY_HPP

#include "../CompressorFactory.hpp"
#include "CompressionContext.hpp"
#include "Compressor.hpp"

namespace module::output::compressor::vaapi {

    class Factory : public CompressorFactory {
    public:
        Factory(const std::string& device, const std::string& driver, const int& quality);
        Factory(const Factory&)     = default;
        Factory(Factory&&) noexcept = default;
        Factory& operator=(const Factory&) = default;
        Factory& operator=(Factory&&) noexcept = default;
        ~Factory() override;

        std::shared_ptr<compressor::Compressor> make_compressor(const uint32_t& width,
                                                                const uint32_t& height,
                                                                const uint32_t& format) override;

    private:
        /// The file descriptor we opened for the DRM device
        int fd{-1};
        /// The compression context that we are using, contains information about the compressor and OpenCL contexts
        CompressionContext cctx{};
        /// The quality that this compressor is configured for
        int quality{0};
    };

}  // namespace module::output::compressor::vaapi

#endif  //  MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_FACTORY_HPP
