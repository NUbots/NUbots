#include "Camera.hpp"

#include <aravis-0.8/arv.h>
#include <cmath>
#include <filesystem>
#include <fmt/format.h>
#include <fstream>

#include "aravis_wrap.hpp"
#include "description_to_fourcc.hpp"
#include "settings.hpp"
#include "time_sync.hpp"

#include "message/input/Image.hpp"
#include "message/input/Sensors.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/fourcc.hpp"
#include "utility/vision/projection.hpp"

namespace module::input {

    using extension::Configuration;
    using message::input::Image;
    using message::input::Sensors;
    using utility::input::ServoID;
    using utility::support::Expression;

    /*************************************************
     *   ____                _              _        *
     *  / ___|___  _ __  ___| |_ __ _ _ __ | |_ ___  *
     * | |   / _ \| '_ \/ __| __/ _` | '_ \| __/ __| *
     * | |__| (_) | | | \__ \ || (_| | | | | |_\__ \ *
     *  \____\___/|_| |_|___/\__\__,_|_| |_|\__|___/ *
     *                                               *
     *************************************************/
    /// The amount of time to observe after recalibrating to work out how long image transfer takes (nanoseconds)
    constexpr int64_t TRANSFER_OFFSET_OBSERVE_TIME = 1e9;
    /// The amount of time on top of the transfer time that would be considered a drifting clock (nanoseconds)
    constexpr int64_t MAX_CLOCK_DRIFT = 5e6;
    /// The over under count of recent messages that that were over the expected value
    constexpr int64_t MAX_COUNT_OVER_TIME_FRAMES = 100;

    /******************************************************************************************************
     *  _____                                _   ____            _                 _   _                  *
     * |  ___|__  _ ____      ____ _ _ __ __| | |  _ \  ___  ___| | __ _ _ __ __ _| |_(_) ___  _ __  ___  *
     * | |_ / _ \| '__\ \ /\ / / _` | '__/ _` | | | | |/ _ \/ __| |/ _` | '__/ _` | __| |/ _ \| '_ \/ __| *
     * |  _| (_) | |   \ V  V / (_| | | | (_| | | |_| |  __/ (__| | (_| | | | (_| | |_| | (_) | | | \__ \ *
     * |_|  \___/|_|    \_/\_/ \__,_|_|  \__,_| |____/ \___|\___|_|\__,_|_|  \__,_|\__|_|\___/|_| |_|___/ *
     *                                                                                                    *
     ******************************************************************************************************/
    void start_streaming(CameraContext& cam);
    void stop_streaming(CameraContext& cam);
    void reset_camera(CameraContext& camera);
    void disable_camera(CameraContext& camera);
    void update_settings(CameraContext& cam);
    CameraContext configure_camera(const Configuration& config, Camera& reactor);

    /**********************************************
     *   ____      _ _ _                _         *
     *  / ___|__ _| | | |__   __ _  ___| | _____  *
     * | |   / _` | | | '_ \ / _` |/ __| |/ / __| *
     * | |__| (_| | | | |_) | (_| | (__|   <\__ \ *
     *  \____\__,_|_|_|_.__/ \__,_|\___|_|\_\___/ *
     *                                            *
     **********************************************/
    void Camera::emit_image(ArvStream* stream, CameraContext* context) {
        using namespace std::chrono;  // NOLINT(google-build-using-namespace) fine in function scope
        // Get now here, as close as possible to when we pop the buffer
        int64_t now       = duration_cast<nanoseconds>(NUClear::clock::now().time_since_epoch()).count();
        ArvBuffer* buffer = arv_stream_try_pop_buffer(stream);
        Camera& reactor   = context->reactor;

        if (buffer != nullptr) {
            if (arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS) {
                // Service the watchdog for this camera
                context->reactor.emit<NUClear::dsl::word::emit::Watchdog>(
                    NUClear::dsl::word::emit::ServiceWatchdog<Camera>(std::string(context->name)));

                int width        = 0;
                int height       = 0;
                size_t data_size = 0;
                arv_buffer_get_image_region(buffer, nullptr, nullptr, &width, &height);
                const auto* data_buffer = reinterpret_cast<const uint8_t*>(arv_buffer_get_data(buffer, &data_size));

                auto& timesync = context->time;

                // Get the timestamp from the buffer
                int64_t ts = arv_buffer_get_timestamp(buffer);

                // If we couldn't calculate the timestamp offset using the cameras API, we have to do it on the fly
                // and hope that our answer isn't too far off the truth
                switch (timesync.type) {
                    case CameraContext::TimeCorrection::LIVE: {
                        // Offset in nanoseconds and seconds
                        int64_t o_ns = (now - ts);

                        // Initialise the filter if this is the first time
                        timesync.offset = timesync.offset == 0 ? o_ns : timesync.offset;

                        // Add process noise
                        timesync.kf.p += timesync.kf.q;

                        // Measurement update
                        double k        = timesync.kf.p / (timesync.kf.p + timesync.kf.r);
                        timesync.offset = std::llround(timesync.offset + k * double(o_ns - timesync.offset));
                        timesync.kf.p   = (1.0 - k) * timesync.kf.p;
                    } break;
                    case CameraContext::TimeCorrection::LATCHED: {
                        // Work out the total amount of time since calibration for both local and remote clocks
                        int64_t total_cam   = ts - timesync.drift.cam_at_calibration;
                        int64_t total_local = now - timesync.drift.local_at_calibration;

                        // Until an appropriate amount of time has passed use the offset to work out what a bad clock
                        // drift would be using exponential smoothing
                        if (total_local < TRANSFER_OFFSET_OBSERVE_TIME) {
                            timesync.drift.max_clock_drift =
                                timesync.drift.max_clock_drift == 0
                                    ? std::abs(total_cam - total_local) + MAX_CLOCK_DRIFT
                                    : (timesync.drift.max_clock_drift * 9 + std::abs(total_cam - total_local)
                                       + MAX_CLOCK_DRIFT)
                                          / 10;
                        }
                        else {
                            // If our camera has drifted out by too large a margin, we recalibrate our clock
                            timesync.drift.over_time_count +=
                                std::abs(total_cam - total_local) > timesync.drift.max_clock_drift
                                    ? 1
                                    : (timesync.drift.over_time_count == 0 ? 0 : -1);

                            // 100 frames have been over time recently
                            if (timesync.drift.over_time_count > MAX_COUNT_OVER_TIME_FRAMES) {
                                reactor.log<NUClear::INFO>(fmt::format(
                                    "Clock drift for {} camera exceeded threshold ({:.1f}ms > {:.1f}ms), recalibrating",
                                    context->name,
                                    double(std::abs(total_cam - total_local)) * 1e-6,
                                    timesync.drift.max_clock_drift * 1e-6));

                                arv::camera_stop_acquisition(context->camera.get());
                                arv_stream_set_emit_signals(context->stream.get(), 0);
                                context->time = sync_clocks(arv_camera_get_device(context->camera.get()));
                                arv_stream_set_emit_signals(context->stream.get(), 1);
                                arv::camera_start_acquisition(context->camera.get());
                            }
                        }
                    } break;
                    case CameraContext::TimeCorrection::PTP: {
                        // Nothing needs to be done as the system clock and the camera clock should already be synced
                        // using ptp
                    } break;
                }

                // Apply our timestamp offset
                ts = ts + context->time.offset;

                auto msg        = std::make_unique<Image>();
                msg->format     = context->fourcc;
                msg->dimensions = Eigen::Matrix<unsigned int, 2, 1>(width, height);
                // TODO(trent) use an std::vector here to avoid the copy
                msg->data.insert(msg->data.end(), data_buffer, data_buffer + data_size);
                msg->id        = context->id;
                msg->name      = context->name;
                msg->timestamp = NUClear::clock::time_point(nanoseconds(ts));

                Eigen::Affine3d Hcw;

                /* Mutex Scope */ {
                    std::lock_guard<std::mutex> lock(reactor.sensors_mutex);

                    Eigen::Affine3d Hpc = context->Hpc;
                    Eigen::Affine3d Hwp;
                    if (reactor.Hwps.empty()) {
                        Hwp = Eigen::Affine3d::Identity();
                    }
                    else {
                        // Find the first time that is not less than the target time
                        auto Hwp_it = std::lower_bound(reactor.Hwps.begin(),
                                                       reactor.Hwps.end(),
                                                       std::make_pair(msg->timestamp, Eigen::Affine3d::Identity()),
                                                       [](const auto& a, const auto& b) { return a.first < b.first; });

                        if (Hwp_it == reactor.Hwps.end()) {
                            // Image is newer than most recent sensors
                            Hwp = std::prev(Hwp_it)->second;
                        }
                        else if (Hwp_it == reactor.Hwps.begin()) {
                            // Image is older than oldest sensors
                            Hwp = Hwp_it->second;
                        }
                        else {
                            // Check Hwp_it and std::prev(Hwp) for closest match
                            Hwp = std::abs((Hwp_it->first - msg->timestamp).count())
                                          < std::abs((std::prev(Hwp_it)->first - msg->timestamp).count())
                                      ? Hwp_it->second
                                      : std::prev(Hwp_it)->second;
                        }
                    }

                    Hcw = Eigen::Affine3d(Hwp * Hpc).inverse();
                }

                msg->lens = context->lens;
                msg->Hcw  = Hcw.matrix();

                reactor.emit(msg);

                // Put the buffer back on the queue
                arv_stream_push_buffer(stream, buffer);
            }
            else {
                std::string msg;
                switch (arv_buffer_get_status(buffer)) {
                    case ARV_BUFFER_STATUS_UNKNOWN: msg = "UNKNOWN"; break;
                    case ARV_BUFFER_STATUS_SUCCESS: msg = "SUCCESS"; break;
                    case ARV_BUFFER_STATUS_CLEARED: msg = "CLEARED"; break;
                    case ARV_BUFFER_STATUS_TIMEOUT: msg = "TIMEOUT"; break;
                    case ARV_BUFFER_STATUS_MISSING_PACKETS: msg = "MISSING PACKETS"; break;
                    case ARV_BUFFER_STATUS_WRONG_PACKET_ID: msg = "WRONG PACKET ID"; break;
                    case ARV_BUFFER_STATUS_SIZE_MISMATCH: msg = "SIZE MISMATCH"; break;
                    case ARV_BUFFER_STATUS_FILLING: msg = "FILLING"; break;
                    case ARV_BUFFER_STATUS_ABORTED: msg = "ABORTED"; break;
                }

                // Put the buffer back on the queue and show an error message
                arv_stream_push_buffer(stream, buffer);
                reactor.log<NUClear::WARN>(
                    fmt::format("Failed getting buffer from the {} camera with error {}", context->name, msg));
            }
        }
    }

    void Camera::control_lost(ArvGvDevice* /*device*/, CameraContext* context) {
        context->reactor.log<NUClear::FATAL>(fmt::format("Control of the {} camera has been lost", context->name));
    }

    /************************************************************************
     *   ____                _          ____                                *
     *  / ___|_ __ ___  __ _| |_ ___   / ___|__ _ _ __ ___   ___ _ __ __ _  *
     * | |   | '__/ _ \/ _` | __/ _ \ | |   / _` | '_ ` _ \ / _ \ '__/ _` | *
     * | |___| | |  __/ (_| | ||  __/ | |__| (_| | | | | | |  __/ | | (_| | *
     *  \____|_|  \___|\__,_|\__\___|  \____\__,_|_| |_| |_|\___|_|  \__,_| *
     *                                                                      *
     ************************************************************************/
    CameraContext configure_camera(const Configuration& config, Camera& reactor) {
        // Find the serial number for this camera
        const auto& serial_number = config["serial_number"].as<std::string>();

        // Strip the .yaml off the name of the file to get the name of the camera
        std::string name = std::filesystem::path(config.path).stem();

        // Find the camera with the correct serial number
        auto find_camera = [](const std::string& name, const std::string& serial_number) {
            unsigned int devices = arv_get_n_devices();
            for (unsigned int i = 0; i < devices; ++i) {
                if (serial_number == arv_get_device_serial_nbr(i)) {
                    return i;
                }
            }
            throw std::runtime_error(fmt::format("{} camera with serial number {} not found", name, serial_number));
        };

        // Find the device, and if we can't check for new devices
        int device_no = 0;
        try {
            device_no = find_camera(name, serial_number);
        }
        catch (const std::runtime_error&) {
            arv_update_device_list();
            device_no = find_camera(name, serial_number);
        }

        // Open the camera: Store as shared pointer
        std::string device_description = arv_get_device_id(device_no);
        auto camera = std::shared_ptr<ArvCamera>(arv::camera_new(device_description), [](ArvCamera* ptr) {
            if (ptr != nullptr && ARV_IS_CAMERA(ptr) == TRUE) {
                g_object_unref(ptr);
            }
        });

        if (ARV_IS_CAMERA(camera.get()) == FALSE) {
            throw std::runtime_error(fmt::format("Failed to create {} camera ({})", name, device_description));
        }

        // Create a new stream object: Store as shared pointer
        auto stream =
            std::shared_ptr<ArvStream>(arv::camera_create_stream(camera.get(), nullptr, nullptr), [](ArvStream* ptr) {
                if (ptr != nullptr && ARV_IS_STREAM(ptr) == TRUE) {
                    g_object_unref(ptr);
                }
            });

        if (ARV_IS_STREAM(stream.get()) == FALSE) {
            throw std::runtime_error(
                fmt::format("Failed to create stream for {} camera ({})", name, device_description));
        }

        // Get a handle to the device pointer for the camera
        auto device = std::shared_ptr<ArvDevice>(arv_camera_get_device(camera.get()), [](ArvDevice* ptr) {
            if (ptr != nullptr && ARV_IS_DEVICE(ptr) == TRUE) {
                g_object_unref(ptr);
            }
        });

        if (ARV_IS_DEVICE(device.get()) == FALSE) {
            throw std::runtime_error(
                fmt::format("Failed to create device for {} camera ({})", name, device_description));
        }

        reactor.log<NUClear::INFO>(fmt::format("Connected to the {} camera ({})", name, device_description));

        // Return camera context
        return CameraContext{reactor,
                             config,
                             name,
                             0,  // fourcc is set later
                             XXH32(name.data(), name.size(), 0x4E55436C),
                             Image::Lens(),      // Lens is constructed in settings
                             Eigen::Affine3d(),  // Hpc is set in settings
                             camera,
                             stream,
                             device,
                             CameraContext::TimeCorrection()};
    }

    /*****************************************************************************
     *   _   _           _       _         ____       _   _   _                  *
     *  | | | |_ __   __| | __ _| |_ ___  / ___|  ___| |_| |_(_)_ __   __ _ ___  *
     *  | | | | '_ \ / _` |/ _` | __/ _ \ \___ \ / _ \ __| __| | '_ \ / _` / __| *
     *  | |_| | |_) | (_| | (_| | ||  __/  ___) |  __/ |_| |_| | | | | (_| \__ \ *
     *   \___/| .__/ \__,_|\__,_|\__\___| |____/ \___|\__|\__|_|_| |_|\__, |___/ *
     *        |_|                                                     |___/      *
     *****************************************************************************/
    void update_settings(CameraContext& cam) {
        const auto& name      = cam.name;
        auto& camera          = cam.camera;
        auto& stream          = cam.stream;
        auto& device          = cam.device;
        Configuration& config = cam.config;

        // Synchronise the clocks
        cam.time = sync_clocks(device.get());

        // Get the fourcc code from the pixel format
        cam.fourcc = description_to_fourcc(config["settings"]["PixelFormat"].as<std::string>());

        // Load Hpc from configuration
        cam.Hpc = Eigen::Matrix4d(config["lens"]["Hpc"].as<Expression>());

        // Apply image offsets to lens_centre, optical axis:
        int full_width  = arv::device_get_integer_feature_value(device.get(), "WidthMax");
        int full_height = arv::device_get_integer_feature_value(device.get(), "HeightMax");

        int offset_x = int(config["settings"]["OffsetX"].as<Expression>());
        int offset_y = int(config["settings"]["OffsetY"].as<Expression>());
        int width    = int(config["settings"]["Width"].as<Expression>());
        int height   = int(config["settings"]["Height"].as<Expression>());

        // Renormalise the focal length
        float focal_length = float(config["lens"]["focal_length"].as<Expression>() * full_width / width);
        float fov          = float(config["lens"]["fov"].as<Expression>());

        // Recentre/renormalise the centre
        Eigen::Vector2f centre =
            (Eigen::Vector2f(config["lens"]["centre"].as<Expression>()) * full_width
             + Eigen::Vector2f(offset_x - (full_width - width) * 0.5, offset_y - (full_height - height) * 0.5))
            / width;

        // Adjust the distortion parameters for the new width units
        Eigen::Vector2f k = config["lens"]["k"].as<Expression>();
        k[0]              = k[0] * std::pow(width / full_width, 2);
        k[1]              = k[1] * std::pow(width / full_width, 4);

        // Set the lens parameters from configuration
        cam.lens = Image::Lens{
            config["lens"]["projection"].as<std::string>(),
            focal_length,
            fov,
            centre,
            k,
        };

        // If the lens fov was auto we need to correct it
        if (!std::isfinite(cam.lens.fov)) {
            const double a = double(height) / double(width);
            std::array<double, 4> options{
                utility::vision::unproject(Eigen::Vector2f(0, 0), cam.lens, Eigen::Vector2f(1, a)).x(),
                utility::vision::unproject(Eigen::Vector2f(1, 0), cam.lens, Eigen::Vector2f(1, a)).x(),
                utility::vision::unproject(Eigen::Vector2f(0, a), cam.lens, Eigen::Vector2f(1, a)).x(),
                utility::vision::unproject(Eigen::Vector2f(1, a), cam.lens, Eigen::Vector2f(1, a)).x()};
            cam.lens.fov = float(std::acos(*std::min_element(options.begin(), options.end())) * 2.0);
        }

        // Apply the region to the camera
        arv::camera_set_region(camera.get(), offset_x, offset_y, width, height);

        // Go through our settings and apply them to the camera
        for (const auto& cfg : config["settings"]) {
            auto key = cfg.first.as<std::string>();

            // Skip the region keys as we handle them above
            if (key == "Width" || key == "Height" || key == "OffsetX" || key == "OffsetY" || key == "PacketSize") {
                continue;
            }

            // Get the feature node
            auto* feature = arv_device_get_feature(device.get(), key.c_str());
            try {
                std::string message;
                if (feature == nullptr) {
                    // Doesn't have this feature
                    NUClear::log<NUClear::ERROR>(
                        fmt::format("The {} camera does not have a setting named {}", name, key));
                }

                // Integer setting and Masked Integer Node Settings
                else if (ARV_IS_GC_INTEGER_NODE(feature) != 0 || ARV_IS_GC_MASKED_INT_REG_NODE(feature) != 0) {
                    auto* setting = reinterpret_cast<ArvGcInteger*>(feature);
                    int64_t value = std::llround(double(cfg.second.as<Expression>()));
                    message       = set_setting(setting, value);
                }

                // Floating point setting and Converter Settings
                else if (ARV_IS_GC_FLOAT_NODE(feature) != 0 || ARV_IS_GC_CONVERTER(feature) != 0) {
                    auto* setting = reinterpret_cast<ArvGcFloat*>(feature);
                    double value  = cfg.second.as<Expression>();
                    message       = set_setting(setting, value);
                }

                // Boolean setting
                else if (ARV_IS_GC_BOOLEAN(feature) != 0) {
                    auto* setting = reinterpret_cast<ArvGcBoolean*>(feature);
                    bool value    = bool(cfg.second.as<Expression>());
                    message       = set_setting(setting, value);
                }

                // Enumeration setting
                else if (ARV_IS_GC_ENUMERATION(feature) != 0) {
                    auto* setting = reinterpret_cast<ArvGcEnumeration*>(feature);
                    auto value    = cfg.second.as<std::string>();
                    message       = set_setting(setting, value);
                }

                else {
                    NUClear::log<NUClear::ERROR>(fmt::format("The type of setting {} is not currently handled", key));
                }

                // If we had a message pass it onto the user
                if (!message.empty()) {
                    NUClear::log<NUClear::INFO>(fmt::format("Updated {} on {} camera: {}", key, name, message));
                }
            }
            catch (const std::runtime_error& e) {
                NUClear::log<NUClear::ERROR>(fmt::format("Error setting {} on {} camera: {}", key, name, e.what()));
            }
        }

        // Check for gigevision device and set packet size to jumbo packets
        if (arv_camera_is_gv_device(camera.get()) != 0) {
            if (YAML::Node mtu = config["settings"]["PacketSize"]) {
                arv::camera_gv_set_packet_size(camera.get(), int(mtu.as<Expression>()));
            }
            g_object_set(stream.get(), "packet-resend", ARV_GV_STREAM_PACKET_RESEND_NEVER, nullptr);
        }

        // Add buffers to the queue
        int payload_size = arv::camera_get_payload(camera.get());
        for (size_t i = 0; i < config["buffer_count"].as<size_t>(); i++) {
            // TODO(trent) Eventually we should use preallocated page aligned data so that we can map directly to
            // the GPU.
            // TODO(trent) Make an std::vector and use it in the buffer to avoid copying to one later
            arv_stream_push_buffer(stream.get(), arv_buffer_new(payload_size, nullptr));
        }
    }

    /****************************************************************************
     *   ____                                  ____            _             _  *
     *  / ___|__ _ _ __ ___   ___ _ __ __ _   / ___|___  _ __ | |_ _ __ ___ | | *
     * | |   / _` | '_ ` _ \ / _ \ '__/ _` | | |   / _ \| '_ \| __| '__/ _ \| | *
     * | |__| (_| | | | | | |  __/ | | (_| | | |__| (_) | | | | |_| | | (_) | | *
     *  \____\__,_|_| |_| |_|\___|_|  \__,_|  \____\___/|_| |_|\__|_|  \___/|_| *
     *                                                                          *
     ****************************************************************************/
    void start_streaming(CameraContext& cam) {
        auto& camera = cam.camera;
        auto& stream = cam.stream;
        auto& device = cam.device;

        // Connect signal events
        if (stream) {
            g_signal_connect(stream.get(), "new-buffer", reinterpret_cast<GCallback>(&Camera::emit_image), &cam);
        }
        if (device) {
            g_signal_connect(device.get(), "control-lost", reinterpret_cast<GCallback>(&Camera::control_lost), &cam);
        }

        // Start aquisition
        if (camera) {
            arv::camera_start_acquisition(camera.get());
        }
        if (stream) {
            arv::stream_set_emit_signals(stream.get(), true);
        }
    }

    void stop_streaming(CameraContext& cam) {
        auto& camera      = cam.camera;
        auto& stream      = cam.stream;
        auto& camera_name = cam.name;

        try {
            if (stream) {
                arv::stream_set_emit_signals(stream.get(), false);
            }
            if (camera) {
                arv::camera_stop_acquisition(camera.get());
            }
        }
        catch (const std::runtime_error&) {
            cam.reactor.log<NUClear::WARN>(fmt::format(
                "Failed to stop acquisition on the {} camera. If we are performing a reset this may be expected.",
                camera_name));
        }
    }

    void disable_camera(CameraContext& camera) {
        // Disable all streams on the camera
        stop_streaming(camera);

        // Release pointers in this order to prevent spurious
        //       g_object_unref: assertion 'G_IS_OBJECT (object)' failed
        // errors
        camera.stream.reset();
        camera.camera.reset();
        camera.device.reset();
    }

    void reset_camera(CameraContext& camera) {
        // Disable the camera
        disable_camera(camera);

        try {
            // Create a new context
            CameraContext new_context = configure_camera(camera.config, camera.reactor);

            // Manually copy context data
            camera.name   = new_context.name;
            camera.fourcc = new_context.fourcc;
            camera.lens   = new_context.lens;
            camera.Hpc    = new_context.Hpc;
            camera.camera = new_context.camera;
            camera.stream = new_context.stream;
            camera.device = new_context.device;
            camera.time   = new_context.time;

            // Update the settings on the camera
            update_settings(camera);

            // Start streaming on the camera
            start_streaming(camera);
        }
        catch (const std::runtime_error& ex) {
            camera.reactor.log<NUClear::WARN>(
                fmt::format("Failed to initiate the {} camera: {}", camera.name, ex.what()));
        }
    }

    Camera::Camera(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Cameras").then("Configuration", [this](const Configuration& config) {
            auto serial_number = config["serial_number"].as<std::string>();

            // The camera already exists update its config and then update its settings
            auto camera = cameras.find(serial_number);
            if (camera != cameras.end()) {
                camera->second.config = config;
                stop_streaming(camera->second);
                update_settings(camera->second);
                start_streaming(camera->second);
            }
            // The camera doesn't exist yet, so create a blank context and create a watchdog
            else {
                const auto camera_name = std::filesystem::path(config.path).stem().string();
                cameras.emplace(std::make_pair(serial_number,
                                               CameraContext{*this,
                                                             config,
                                                             camera_name,
                                                             0,  // fourcc is set later
                                                             XXH32(camera_name.data(), camera_name.size(), 0x4E55436C),
                                                             Image::Lens(),      // Lens is constructed in settings
                                                             Eigen::Affine3d(),  // Hpc is set in settings
                                                             nullptr,
                                                             nullptr,
                                                             nullptr,
                                                             CameraContext::TimeCorrection()}));

                on<Watchdog<Camera, 1, std::chrono::seconds>, Single>(camera_name).then([this, serial_number] {
                    auto camera = cameras.find(serial_number);
                    log<NUClear::WARN>(
                        fmt::format("The {} camera is showing no activity. Performing a reset.", camera->second.name));
                    reset_camera(camera->second);
                });
            }
        });

        on<Trigger<Sensors>>().then("Buffer Sensors", [this](const Sensors& sensors) {
            std::lock_guard<std::mutex> lock(sensors_mutex);
            auto now = NUClear::clock::now();
            Hwps.resize(std::distance(Hwps.begin(), std::remove_if(Hwps.begin(), Hwps.end(), [now](const auto& v) {
                                          return v.first < (now - std::chrono::milliseconds(500));
                                      })));

            // Get torso to head, and torso to world
            Eigen::Affine3d Htp(sensors.Htx[ServoID::HEAD_PITCH]);
            Eigen::Affine3d Htw(sensors.Htw);
            Eigen::Affine3d Hwp = Htw.inverse() * Htp;

            Hwps.emplace_back(sensors.timestamp, Hwp);
        });


        on<Shutdown>().then([this] {
            for (auto& camera : cameras) {
                // Disable the camera
                disable_camera(camera.second);
            }
            arv_shutdown();
            cameras.clear();
        });
    }
}  // namespace module::input
