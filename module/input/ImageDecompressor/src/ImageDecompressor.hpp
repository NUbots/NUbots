#ifndef MODULE_INPUT_IMAGEDECOMPRESSOR_HPP
#define MODULE_INPUT_IMAGEDECOMPRESSOR_HPP

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <nuclear>

#include "decompressor/DecompressorFactory.hpp"

namespace module::input {

    class ImageDecompressor : public NUClear::Reactor {
    private:
        struct DecompressorContext {
            struct Decompressor {
                /// This atomic bool gets turned on/off depending on if it is in use
                std::unique_ptr<std::atomic<bool>> active;
                /// The actual decompressor that will convert the image to jpeg format
                std::shared_ptr<decompressor::Decompressor> decompressor;
            };

            /// A list of decompressors that can be used
            std::vector<Decompressor> decompressors;

            /// The format so that if these change we can regenerate the decompressors
            uint32_t width{};
            uint32_t height{};
            uint32_t format{};
        };

    public:
        /// @brief Called by the powerplant to build and setup the ImageDecompressor reactor.
        explicit ImageDecompressor(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct {
            /// The decompressor factories that will be used to create a decompression context
            std::vector<std::pair<std::shared_ptr<decompressor::DecompressorFactory>, int>> factories;
        } config;

        /// The decompressors that are available for use
        std::mutex decompressor_mutex;
        std::map<uint32_t, std::shared_ptr<DecompressorContext>> decompressors;

        /// Number of images that have been decompressed since this was last reset
        int decompressed = 0;
        /// Number of images that have been dropped since this was last reset
        int dropped = 0;
    };

}  // namespace module::input

#endif  // MODULE_INPUT_IMAGEDECOMPRESSOR_HPP
