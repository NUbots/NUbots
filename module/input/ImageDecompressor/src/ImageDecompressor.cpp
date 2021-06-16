#include "ImageDecompressor.hpp"

#include <fmt/format.h>

#include "decompressor/turbojpeg/Factory.hpp"

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
#include "message/output/CompressedImage.hpp"

namespace module::input {

    using extension::Configuration;
    using message::input::Image;
    using message::output::CompressedImage;

    ImageDecompressor::ImageDecompressor(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("ImageDecompressor.yaml").then("Configure Decompressors", [this](const Configuration& cfg) {
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();

            // Clear the compressors and factories
            std::lock_guard<std::mutex> lock(decompressor_mutex);
            decompressors.clear();
            config.factories.clear();

            for (const auto& c : cfg["decompressors"].config) {
                if (c["name"].as<std::string>() == "turbojpeg") {
                    config.factories.emplace_back(std::make_shared<decompressor::turbojpeg::Factory>(),
                                                  c["concurrent"].as<int>());
                }
            }
        });

        on<Trigger<CompressedImage>>().then("Decompress Image", [this](const CompressedImage& image) {
            // Find our list of compressors, and if needed recreate it
            std::shared_ptr<DecompressorContext> ctx;

            /* Mutex Scope */ {
                std::lock_guard<std::mutex> lock(decompressor_mutex);
                auto it = decompressors.find(image.id);
                if (it == decompressors.end() || it->second->width != image.dimensions[0]
                    || it->second->height != image.dimensions[1] || it->second->format != image.format) {
                    log<NUClear::INFO>("Rebuilding decompressors for", image.name, "camera");

                    // Replace the existing one with a new one
                    it = decompressors.insert(std::make_pair(image.id, std::make_shared<DecompressorContext>())).first;
                    it->second->width  = image.dimensions[0];
                    it->second->height = image.dimensions[1];
                    it->second->format = image.format;

                    for (auto& f : config.factories) {
                        for (int i = 0; i < f.second; ++i) {
                            auto a = std::make_unique<std::atomic<bool>>();
                            it->second->decompressors.emplace_back(DecompressorContext::Decompressor{
                                std::move(a),
                                f.first->make_decompressor(image.dimensions[0], image.dimensions[1], image.format),
                            });
                        }
                    }
                }
                ctx = it->second;
            }

            // Look through our compressors and try to find the first free one
            for (auto& ctx : ctx->decompressors) {
                // We swap in true to the atomic and if we got false back then it wasn't active previously
                if (!ctx.active->exchange(true)) {
                    std::exception_ptr eptr;
                    try {
                        auto msg = std::make_unique<Image>();

                        // Compress the data
                        auto result = ctx.decompressor->decompress(image.data);
                        msg->data   = result.first;
                        msg->format = result.second;

                        // Copy across the other attributes
                        msg->dimensions        = image.dimensions;
                        msg->id                = image.id;
                        msg->name              = image.name;
                        msg->timestamp         = image.timestamp;
                        msg->Hcw               = image.Hcw;
                        msg->lens.projection   = int(image.lens.projection);
                        msg->lens.focal_length = image.lens.focal_length;
                        msg->lens.fov          = image.lens.fov;
                        msg->lens.centre       = image.lens.centre;
                        msg->lens.k            = image.lens.k;

                        // Emit the compressed image
                        emit(msg);
                    }
                    catch (...) {
                        eptr = std::current_exception();
                    }

                    // This sets the atomic integer back to false so another thread can use this compressor
                    ctx.active->store(false);

                    if (eptr) {
                        // Exception :(
                        std::rethrow_exception(eptr);
                    }
                    else {
                        // Successful compression!
                        ++decompressed;
                        return;
                    }
                }
            }
            // We failed to decompress this image
            ++dropped;
        });

        on<Every<1, std::chrono::seconds>>().then("Stats", [this] {
            log<NUClear::DEBUG>(fmt::format("Receiving {}/s, Decompressing {}/s,  Dropping {}/s ({}%)",
                                            decompressed + dropped,
                                            decompressed,
                                            dropped,
                                            100 * double(decompressed) / double(decompressed + dropped)));
            decompressed = 0;
            dropped      = 0;
        });
    }

}  // namespace module::input
