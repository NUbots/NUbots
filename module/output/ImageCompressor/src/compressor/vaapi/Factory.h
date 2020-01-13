#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_FACTORY_H
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_FACTORY_H

#include "../CompressorFactory.h"
#include "CompressionContext.h"
#include "Compressor.h"

namespace module::output::compressor::vaapi {

class Factory : public CompressorFactory {
public:
    Factory(const std::string& device, const std::string& driver, const int& quality);
    virtual ~Factory();

    virtual std::shared_ptr<compressor::Compressor> make_compressor(const uint32_t width,
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

#endif  //  MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_FACTORY_H
