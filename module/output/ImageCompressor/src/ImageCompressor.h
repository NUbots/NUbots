#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_H
#define MODULE_OUTPUT_IMAGECOMPRESSOR_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <nuclear>

#include "compressor/CompressorFactory.h"

namespace module {
namespace output {

    class ImageCompressor : public NUClear::Reactor {
    private:
        struct CompressorContext {
            struct Compressor {
                /// This atomic bool gets turned on/off depending on if it is in use
                std::unique_ptr<std::atomic<bool>> active;
                /// The actual compressor that will convert the image to jpeg format
                std::shared_ptr<compressor::Compressor> compressor;
            };

            /// A list of compressors that can be used
            std::vector<Compressor> compressors;

            /// The width height and format so that if these change we can regenerate the compressors
            uint32_t width;
            uint32_t height;
            uint32_t format;
        };

    public:
        /// @brief Called by the powerplant to build and setup the ImageCompressor reactor.
        explicit ImageCompressor(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct {
            /// The compressor factories that will be used to create a compression context
            std::vector<std::pair<std::shared_ptr<compressor::CompressorFactory>, int>> factories;
        } config;

        /// The compressors that are available for use
        std::mutex compressor_mutex;
        std::map<uint32_t, std::shared_ptr<CompressorContext>> compressors;

        /// Number of images that have been compressed since this was last reset
        int compressed = 0;
        /// Number of images that have been dropped since this was last reset
        int dropped = 0;
    };

}  // namespace output
}  // namespace module

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_H
