/*
 * MIT License
 *
 * Copyright (c) 2019 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "ImageCompressor.hpp"

#include <fmt/format.h>

#include "compressor/turbojpeg/Factory.hpp"
#include "compressor/vaapi/Factory.hpp"

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
#include "message/output/CompressedImage.hpp"

#include "utility/vision/fourcc.hpp"

namespace module::output {

    using extension::Configuration;
    using message::input::Image;
    using message::output::CompressedImage;

    /**
     * Get the new fourcc code that we will use to describe this image after it has been compressed
     */
    uint32_t compressed_fourcc(const uint32_t old_fourcc) {
        using utility::vision::fourcc;
        switch (old_fourcc) {
            case fourcc("BGGR"): return fourcc("JPBG");  // Permuted Bayer
            case fourcc("RGGB"): return fourcc("JPRG");  // Permuted Bayer
            case fourcc("GRBG"): return fourcc("JPGR");  // Permuted Bayer
            case fourcc("GBRG"): return fourcc("JPGB");  // Permuted Bayer

            case fourcc("PBG8"): return fourcc("PJBG");  // Polarized Colour
            case fourcc("PRG8"): return fourcc("PJRG");  // Polarized Colour
            case fourcc("PGR8"): return fourcc("PJGR");  // Polarized Colour
            case fourcc("PGB8"): return fourcc("PJGB");  // Polarized Colour

            case fourcc("PY8 "): return fourcc("PJPG");  // Polarized Monochrome

            case fourcc("BGRA"):                         // RGB formats
            case fourcc("BGR8"):                         // RGB formats
            case fourcc("BGR3"):                         // RGB formats
            case fourcc("RGBA"):                         // RGB formats
            case fourcc("RGB8"):                         // RGB formats
            case fourcc("RGB3"):                         // RGB formats
            case fourcc("GRAY"):                         // Monochrome formats
            case fourcc("GREY"):                         // Monochrome formats
            case fourcc("Y8  "):                         // Monochrome formats
            case fourcc("Y16 "): return fourcc("JPEG");  // Monochrome formats

            default: throw std::runtime_error("Unhandled format");
        }
    }

    ImageCompressor::ImageCompressor(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("ImageCompressor.yaml").then("Configure Compressors", [this](const Configuration& cfg) {
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();

            // Clear the compressors and factories
            std::lock_guard<std::mutex> lock(compressor_mutex);
            compressors.clear();
            config.factories.clear();

            for (const auto& c : cfg["compressors"].config) {
                if (c["name"].as<std::string>() == "vaapi") {
                    config.factories.emplace_back(
                        std::make_shared<compressor::vaapi::Factory>(c["device"].as<std::string>(),
                                                                     c["driver"].as<std::string>(),
                                                                     c["quality"].as<int>()),
                        c["concurrent"].as<int>());
                }
                else if (c["name"].as<std::string>() == "turbojpeg") {
                    config.factories.emplace_back(
                        std::make_shared<compressor::turbojpeg::Factory>(c["quality"].as<int>()),
                        c["concurrent"].as<int>());
                }
            }
        });

        on<Trigger<Image>>().then("Compress Image", [this](const Image& image) {
            // Find our list of compressors, and if needed recreate it
            std::shared_ptr<CompressorContext> ctx;

            /* Mutex Scope */ {
                std::lock_guard<std::mutex> lock(compressor_mutex);
                auto it = compressors.find(image.id);
                if (it == compressors.end() || it->second->width != image.dimensions[0]
                    || it->second->height != image.dimensions[1] || it->second->format != image.format) {
                    log<NUClear::INFO>("Rebuilding compressors for", image.name, "camera");

                    // Replace the existing one with a new one
                    it = compressors.insert(std::make_pair(image.id, std::make_shared<CompressorContext>())).first;
                    it->second->width  = image.dimensions[0];
                    it->second->height = image.dimensions[1];
                    it->second->format = image.format;

                    for (auto& f : config.factories) {
                        for (int i = 0; i < f.second; ++i) {
                            it->second->compressors.emplace_back(CompressorContext::Compressor{
                                std::make_unique<std::mutex>(),
                                f.first->make_compressor(image.dimensions[0], image.dimensions[1], image.format),
                            });
                        }
                    }
                }
                ctx = it->second;
            }

            // Look through our compressors and try to find the first free one
            for (auto& ctx : ctx->compressors) {
                // Attempt to acquire a lock on the mutex, if this succeeds then the context wasn't being used
                std::unique_lock lock(*ctx.mutex, std::try_to_lock);
                if (lock) {

                    auto msg = std::make_unique<CompressedImage>();

                    // Compress the data
                    msg->data = ctx.compressor->compress(image.data);

                    // The format depends on what kind of data we took in
                    msg->format = compressed_fourcc(image.format);

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

                    // Successful compression!
                    ++compressed;
                    return;
                }
            }
            // We failed to compress this image
            ++dropped;
        });

        on<Every<1, std::chrono::seconds>>().then("Stats", [this] {
            log<NUClear::DEBUG>(fmt::format("Receiving {}/s, Compressing {}/s,  Dropping {}/s ({}%)",
                                            compressed + dropped,
                                            compressed,
                                            dropped,
                                            100 * double(compressed) / double(compressed + dropped)));
            compressed = 0;
            dropped    = 0;
        });
    }

}  // namespace module::output
