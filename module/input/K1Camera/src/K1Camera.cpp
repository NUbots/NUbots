/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
#include "K1Camera.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <chrono>
#include <cstring>
#include <fmt/format.h>
#include <thread>

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"

#include "utility/vision/fourcc.hpp"

namespace bip = boost::interprocess;

namespace module::input {

    using extension::Configuration;
    using message::input::Image;
    using utility::vision::fourcc;

    // Must match the writer-side layout in NUbridge exactly (binary compatibility).
    struct SharedImageHeader {
        bip::interprocess_mutex     mutex;
        bip::interprocess_condition has_new_frame;
        uint64_t sequence{0};   // increments each frame
        uint32_t data_size{0};  // bytes of pixel data following this header
        uint32_t width{0};
        uint32_t height{0};
        char     encoding[32]{};  // ROS2 encoding string, e.g. "rgb8"
        float focal_length {0.0};
        float fov {0.0};
        float centre_x {0.0};
        float centre_y {0.0};
        float k1 {0.0};
        float k2 {0.0};
    };

    static constexpr std::size_t MAX_IMAGE_BYTES = 2 * 1024 * 1024;

    static uint32_t ros_encoding_to_fourcc(const std::string& enc) {
        // clang-format off
        if (enc == "rgb8")         return fourcc("RGB8");
        if (enc == "bgr8")         return fourcc("BGR8");
        if (enc == "mono8")        return fourcc("GREY");
        if (enc == "bayer_bggr8")  return fourcc("BGGR");
        if (enc == "bayer_grbg8")  return fourcc("GRBG");
        if (enc == "bayer_rggb8")  return fourcc("RGGB");
        if (enc == "bayer_gbrg8")  return fourcc("GBRG");
        // clang-format on
        return 0;
    }

    void K1Camera::stop_cameras() {
        std::unique_lock<std::mutex> lock(cameras_mutex);

        for (auto& cam : cameras) {
            cam->running = false;
        }
        // Join without holding the mutex so camera threads can complete cleanly.
        // Threads check running on each timed_wait timeout (≤100 ms), so joins are fast.
        auto local_cameras = std::move(cameras);
        lock.unlock();

        for (auto& cam : local_cameras) {
            if (cam->thread.joinable()) {
                cam->thread.join();
            }
        }
    }

    void K1Camera::camera_thread(CameraContext& ctx) {
        // Outer loop: retry until the segment is available (NUbridge may start after NUbots).
        while (ctx.running) {
            try {
                // read_write is required even for the reader because interprocess_mutex and
                // interprocess_condition must modify their internal state on lock/wait.
                bip::shared_memory_object shm(bip::open_only, ctx.segment_name.c_str(), bip::read_write);
                bip::mapped_region        region(shm, bip::read_write);

                auto*       header = reinterpret_cast<SharedImageHeader*>(region.get_address());
                const auto* pixels = reinterpret_cast<const uint8_t*>(header + 1);

                uint64_t last_sequence = 0;

                // Inner loop: read frames until shutdown or segment disappears.
                while (ctx.running) {
                    uint64_t    seq          = 0;
                    uint32_t    data_size    = 0;
                    uint32_t    width        = 0;
                    uint32_t    height       = 0;
                    uint32_t    format       = 0;
                    float       focal_length = 0.0f;
                    float       fov          = 0.0f;
                    float       centre_x     = 0.0f;
                    float       centre_y     = 0.0f;
                    float       k1           = 0.0f;
                    float       k2           = 0.0f;
                    std::string data;

                    {
                        bip::scoped_lock<bip::interprocess_mutex> lock(header->mutex);

                        // Use timed_wait so the running flag is rechecked every 100 ms
                        // even if the writer stalls.
                        while (ctx.running && header->sequence == last_sequence) {
                            header->has_new_frame.timed_wait(
                                lock,
                                boost::posix_time::microsec_clock::universal_time()
                                    + boost::posix_time::milliseconds(100));
                        }

                        if (!ctx.running) {
                            break;
                        }

                        seq       = header->sequence;
                        data_size = header->data_size;
                        width     = header->width;
                        height    = header->height;

                        const std::string encoding(header->encoding,
                                                   strnlen(header->encoding, sizeof(header->encoding)));
                        format = ros_encoding_to_fourcc(encoding);

                        if (format == 0) {
                            log<WARN>(fmt::format("K1Camera: {} unknown encoding '{}'",
                                                  ctx.segment_name,
                                                  encoding));
                            last_sequence = seq;
                            continue;
                        }

                        if (data_size > MAX_IMAGE_BYTES) {
                            log<WARN>(fmt::format("K1Camera: {} data_size {} exceeds MAX_IMAGE_BYTES",
                                                  ctx.segment_name,
                                                  data_size));
                            last_sequence = seq;
                            continue;
                        }

                        focal_length = header->focal_length;
                        fov          = header->fov;
                        centre_x     = header->centre_x;
                        centre_y     = header->centre_y;
                        k1           = header->k1;
                        k2           = header->k2;

                        // Copy pixel data while holding the lock; release before emit.
                        data.assign(reinterpret_cast<const char*>(pixels), data_size);
                    }

                    if (last_sequence != 0 && seq != last_sequence + 1) {
                        log<DEBUG>(fmt::format("K1Camera: {} dropped {} frame(s)",
                                               ctx.segment_name,
                                               seq - last_sequence - 1));
                    }
                    last_sequence = seq;

                    auto msg = std::make_unique<Image>();
                    msg->format          = format;
                    msg->dimensions.x()  = width;
                    msg->dimensions.y()  = height;
                    msg->data            = std::move(data);
                    msg->id              = ctx.id;
                    msg->name            = ctx.segment_name;
                    msg->timestamp       = NUClear::clock::now();
                    msg->lens.projection = Image::Lens::RECTILINEAR;
                    msg->lens.focal_length       = focal_length;
                    msg->lens.fov                = fov;
                    msg->lens.centre             = {centre_x, centre_y};
                    msg->lens.k                  = {k1, k2};

                    emit(msg);
                }
            }
            catch (const bip::interprocess_exception& ex) {
                if (ctx.running) {
                    log<WARN>(fmt::format("K1Camera: segment '{}' unavailable: {} — retrying in 500 ms",
                                          ctx.segment_name,
                                          ex.what()));
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            }
        }
    }

    K1Camera::K1Camera(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("K1Camera.yaml").then([this](const Configuration& cfg) {
            stop_cameras();

            std::lock_guard<std::mutex> lock(cameras_mutex);
            for (const auto& entry : cfg["cameras"]) {
                auto ctx          = std::make_unique<CameraContext>();
                ctx->segment_name = entry["segment"].as<std::string>();
                ctx->id           = entry["id"].as<uint32_t>();
                ctx->running      = true;
                ctx->thread       = std::thread([this, raw = ctx.get()] { camera_thread(*raw); });
                cameras.push_back(std::move(ctx));
            }
        });

        on<Shutdown>().then([this] { stop_cameras(); });
    }

    K1Camera::~K1Camera() {
        stop_cameras();
    }

}  // namespace module::input
