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

#include <algorithm>
#include <booster/common/dds/dds_entity.hpp>
#include <chrono>
#include <cmath>
#include <fmt/format.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/platform/Booster/channel_factory.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/fourcc.hpp"

namespace module::input {

    using extension::Configuration;
    using message::input::Image;
    using message::input::Sensors;
    using utility::nusight::graph;
    using utility::platform::Booster::ensure_channel_factory;
    using utility::support::Expression;
    using utility::vision::fourcc;

    using booster::common::DdsExecutorDispatchMode;
    using booster::common::DdsExecutorOverflowPolicy;
    using booster::common::DdsReaderExecutorOptions;
    using booster::robot::ChannelFactory;

    /// @brief How stale a Hcw may be before it is dropped from the buffer.
    static constexpr std::chrono::milliseconds HCW_BUFFER_DURATION{500};
    /// @brief How far a capture timestamp may be from our own clock before we distrust it.
    static constexpr std::chrono::seconds MAX_CLOCK_SKEW{1};
    /// @brief How long to wait for CameraInfo before warning that the fallback lens is in use.
    static constexpr std::chrono::seconds CAMERA_INFO_TIMEOUT{5};

    /// @brief Map a ROS sensor_msgs/Image encoding string to the FOURCC code NUbots uses.
    static uint32_t ros_encoding_to_fourcc(const std::string& enc) {
        // clang-format off
        if (enc == "rgb8")         return fourcc("RGB3");
        if (enc == "bgr8")         return fourcc("BGR3");
        if (enc == "mono8")        return fourcc("GREY");
        if (enc == "bayer_bggr8")  return fourcc("BGGR");
        if (enc == "bayer_grbg8")  return fourcc("GRBG");
        if (enc == "bayer_rggb8")  return fourcc("RGGB");
        if (enc == "bayer_gbrg8")  return fourcc("GBRG");
        if (enc == "nv12")         return fourcc("NV12");
        // clang-format on
        return 0;
    }

    Eigen::Isometry3d K1Camera::nearest_Hcw(const NUClear::clock::time_point& timestamp) {
        std::lock_guard<std::mutex> lock(sensors_mutex);

        if (Hcws.empty()) {
            return Eigen::Isometry3d::Identity();
        }

        auto it = std::lower_bound(Hcws.begin(), Hcws.end(), timestamp, [](const auto& a, const auto& t) {
            return a.first < t;
        });

        if (it == Hcws.end()) {
            return std::prev(it)->second;
        }
        if (it == Hcws.begin()) {
            return it->second;
        }
        // Pick whichever of the two straddling samples is closer in time
        return std::abs((it->first - timestamp).count()) < std::abs((std::prev(it)->first - timestamp).count())
                   ? it->second
                   : std::prev(it)->second;
    }

    void K1Camera::camera_info_handler(CameraContext& ctx, const void* msg) {
        const auto* info = static_cast<const sensor_msgs::msg::CameraInfo*>(msg);

        const auto& k = info->k();
        if (k.size() < 6 || k[0] == 0.0) {
            log<WARN>(fmt::format("Ignoring CameraInfo on '{}' with an unusable intrinsic matrix", ctx.info_topic));
            return;
        }

        const float fx = static_cast<float>(k[0]);
        const float cx = static_cast<float>(k[2]);
        const float cy = static_cast<float>(k[5]);
        const float w  = static_cast<float>(info->width());
        const float h  = static_cast<float>(info->height());

        if (w == 0.0f || h == 0.0f) {
            log<WARN>(fmt::format("Ignoring CameraInfo on '{}' with zero dimensions", ctx.info_topic));
            return;
        }

        // message::input::Image::Lens stores focal_length and centre normalised by image width
        Image::Lens lens;
        lens.projection   = Image::Lens::Projection::RECTILINEAR;
        lens.focal_length = fx / w;
        lens.fov          = 2.0f * std::atan(w / (2.0f * fx));
        lens.centre       = Eigen::Vector2f((cx - w / 2.0f) / w, (cy - h / 2.0f) / w);
        lens.k            = Eigen::Vector2f(info->d().size() > 0 ? static_cast<float>(info->d()[0]) : 0.0f,
                                 info->d().size() > 1 ? static_cast<float>(info->d()[1]) : 0.0f);

        {
            std::lock_guard<std::mutex> lock(ctx.lens_mutex);
            ctx.lens = lens;
        }

        if (!ctx.have_camera_info.exchange(true)) {
            log<INFO>(fmt::format("Got CameraInfo for '{}': focal_length {:.4f}, fov {:.3f} rad, centre ({:.4f},"
                                  " {:.4f})",
                                  ctx.camera_name,
                                  lens.focal_length,
                                  lens.fov,
                                  lens.centre.x(),
                                  lens.centre.y()));
        }
    }

    NUClear::clock::time_point K1Camera::capture_time(CameraContext& ctx, const sensor_msgs::msg::Image& image) {
        // Prefer the capture timestamp from the camera over our receive time, so the image is
        // matched against the kinematics from when it was actually taken
        const auto& stamp = image.header().stamp();
        const auto now    = NUClear::clock::now();
        const auto stamped =
            NUClear::clock::time_point(std::chrono::duration_cast<NUClear::clock::duration>(
                std::chrono::seconds(stamp.sec()) + std::chrono::nanoseconds(stamp.nanosec())));

        const auto skew = std::chrono::abs(stamped - now);
        if (stamp.sec() == 0 || skew > MAX_CLOCK_SKEW) {
            if (!ctx.warned_about_timestamp.exchange(true)) {
                log<WARN>(fmt::format("Capture timestamps on '{}' are {} s from our clock, using receive time"
                                      " instead. Check that BoosterOS and NUbots share a clock.",
                                      ctx.image_topic,
                                      std::chrono::duration_cast<std::chrono::duration<double>>(skew).count()));
            }
            return now;
        }
        return stamped;
    }

    void K1Camera::image_handler(CameraContext& ctx, const void* msg) {
        const auto* image = static_cast<const sensor_msgs::msg::Image*>(msg);

        const uint32_t width  = image->width();
        const uint32_t height = image->height();
        const auto& bytes     = image->data();

        if (width == 0 || height == 0 || bytes.empty()) {
            log<WARN>(fmt::format("Dropping empty frame from '{}'", ctx.camera_name));
            return;
        }

        uint32_t format = ros_encoding_to_fourcc(image->encoding());
        if (format == 0) {
            log<WARN>(fmt::format("Unknown encoding '{}' on '{}'", image->encoding(), ctx.image_topic));
            return;
        }

        auto out = std::make_unique<Image>();

        // Convert NV12 to RGB3
        if (format == fourcc("NV12")) {
            const std::size_t expected = std::size_t(width) * height * 3 / 2;
            if (bytes.size() < expected) {
                log<WARN>(fmt::format("Dropping truncated NV12 frame from '{}': {} bytes, expected {}",
                                      ctx.camera_name,
                                      bytes.size(),
                                      expected));
                return;
            }
            // Size the destination first and point a Mat at it, so cvtColor writes the converted
            // pixels straight into the message
            out->data.resize(std::size_t(width) * height * 3);
            const cv::Mat nv12(int(height * 3 / 2), int(width), CV_8UC1, const_cast<uint8_t*>(bytes.data()));
            cv::Mat rgb(int(height), int(width), CV_8UC3, out->data.data());
            cv::cvtColor(nv12, rgb, cv::COLOR_YUV2RGB_NV12);
            format = fourcc("RGB3");
        }
        else {
            out->data.assign(bytes.begin(), bytes.end());
        }

        const auto timestamp = capture_time(ctx, *image);

        out->format         = format;
        out->dimensions.x() = width;
        out->dimensions.y() = height;
        out->id             = ctx.id;
        out->name           = ctx.camera_name;
        out->timestamp      = timestamp;
        {
            std::lock_guard<std::mutex> lock(ctx.lens_mutex);
            out->lens = ctx.lens;
        }
        out->Hcw = nearest_Hcw(timestamp);

        emit(graph("Camera Pose", out->Hcw.translation().x(), out->Hcw.translation().y(), out->Hcw.translation().z()));
        emit(out);
    }

    K1Camera::K1Camera(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("K1Camera.yaml").then([this](const Configuration& cfg) {
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();

            // Can't reconfigure the readers once the topics have been subscribed to
            if (channels_created) {
                log<WARN>("Camera topics changed but readers already exist, restart to apply");
                return;
            }

            cameras.clear();
            for (const auto& entry : cfg["cameras"]) {
                auto ctx         = std::make_unique<CameraContext>();
                ctx->image_topic = entry["topic"].as<std::string>();
                ctx->info_topic  = ctx->image_topic + "/camera_info";
                ctx->camera_name = entry["name"].as<std::string>();
                ctx->id          = entry["id"].as<uint32_t>();

                cameras.push_back(std::move(ctx));
            }
        });

        on<Startup>().then("Subscribe to cameras", [this] {
            ensure_channel_factory();

            // Setup DDS reader options
            DdsReaderExecutorOptions image_options;
            image_options.queue_capacity  = 1;
            image_options.overflow_policy = DdsExecutorOverflowPolicy::kLatestOnly;
            image_options.dispatch_mode   = DdsExecutorDispatchMode::kDedicated;

            for (auto& ctx : cameras) {
                log<INFO>(fmt::format("Subscribing to '{}' for camera '{}'", ctx->image_topic, ctx->camera_name));
                ctx->image_channel = ChannelFactory::Instance()->CreateRecvChannel<sensor_msgs::msg::Image>(
                    ctx->image_topic,
                    [this, raw = ctx.get()](const void* msg) { image_handler(*raw, msg); },
                    /* reliable = */ false,
                    image_options);

                // CameraInfo is low rate and we need every update, so take it reliably
                ctx->info_channel = ChannelFactory::Instance()->CreateRecvChannel<sensor_msgs::msg::CameraInfo>(
                    ctx->info_topic,
                    [this, raw = ctx.get()](const void* msg) { camera_info_handler(*raw, msg); },
                    /* reliable = */ true);
            }

            channels_created = true;
        });

        on<Trigger<Sensors>>().then("Buffer Hcw", [this](const Sensors& sensors) {
            std::lock_guard<std::mutex> lock(sensors_mutex);
            const auto cutoff = NUClear::clock::now() - HCW_BUFFER_DURATION;
            while (!Hcws.empty() && Hcws.front().first < cutoff) {
                Hcws.pop_front();
            }
            Hcws.emplace_back(sensors.timestamp, Eigen::Isometry3d(sensors.Hcw));
        });

        on<Shutdown>().then([this] {
            log<INFO>("Closing camera channels");
            for (auto& ctx : cameras) {
                if (ctx->image_channel != nullptr) {
                    ChannelFactory::Instance()->CloseReader(ctx->image_topic);
                }
                if (ctx->info_channel != nullptr) {
                    ChannelFactory::Instance()->CloseReader(ctx->info_topic);
                }
            }
        });
    }

}  // namespace module::input
