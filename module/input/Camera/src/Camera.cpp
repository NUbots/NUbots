#include "Camera.h"

extern "C" {
#include <aravis-0.8/arv.h>
}

#include <fmt/format.h>

#include <cmath>

#include "description_to_fourcc.h"
#include "message/input/Image.h"
#include "message/input/Sensors.h"
#include "settings.h"
#include "time_sync.h"
#include "utility/input/ServoID.h"
#include "utility/support/yaml_expression.h"
#include "utility/vision/fourcc.h"
#include "utility/vision/projection.h"

namespace module {
namespace input {

    using extension::Configuration;
    using message::input::Image;
    using message::input::Sensors;
    using utility::input::ServoID;
    using utility::support::Expression;
    using utility::vision::fourcc;

    /// The amount of time to observe after recalibrating to work out how long image transfer takes (nanoseconds)
    constexpr int64_t TRANSFER_OFFSET_OBSERVE_TIME = 1e9;
    /// The amount of time on top of the transfer time that would be considered a drifting clock (nanoseconds)
    constexpr int64_t MAX_CLOCK_DRIFT = 5e6;
    /// The over under count of recent messages that that were over the expected value
    constexpr int64_t MAX_COUNT_OVER_TIME_FRAMES = 100;

    Camera::Camera(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Cameras").then("Configuration", [this](const Configuration& config) {
            std::string serial_number = config["serial_number"].as<std::string>();

            // Find the camera if it has already been loaded
            auto it = cameras.find(serial_number);

            /************************************************************************
             *   ____                _          ____                                *
             *  / ___|_ __ ___  __ _| |_ ___   / ___|__ _ _ __ ___   ___ _ __ __ _  *
             * | |   | '__/ _ \/ _` | __/ _ \ | |   / _` | '_ ` _ \ / _ \ '__/ _` | *
             * | |___| | |  __/ (_| | ||  __/ | |__| (_| | | | | | |  __/ | | (_| | *
             *  \____|_|  \___|\__,_|\__\___|  \____\__,_|_| |_| |_|\___|_|  \__,_| *
             *                                                                      *
             ************************************************************************/
            if (it == cameras.end()) {
                // Strip the .yaml off the name of the file to get the name of the camera
                std::string name = ::basename(config.fileName.substr(0, config.fileName.find_last_of('.')).c_str());

                // Find the camera with the correct serial number
                auto find_camera = [](const std::string& serial_number) {
                    int devices = arv_get_n_devices();
                    for (int i = 0; i < devices; ++i) {
                        if (serial_number.compare(arv_get_device_serial_nbr(i)) == 0) {
                            return i;
                        }
                    }
                    return -1;
                };

                // Find the device, and if we can't check for new devices
                int device_no = find_camera(serial_number);
                if (device_no == -1) {
                    arv_update_device_list();
                    device_no = find_camera(serial_number);
                }

                // If device_no is still -1 throw an error
                if (device_no == -1) {
                    throw std::runtime_error(
                        fmt::format("{} camera with serial number {} not found", name, serial_number));
                }
                else {

                    // Open the camera: Store as shared pointer
                    std::string device_description = arv_get_device_id(device_no);
                    auto camera =
                        std::shared_ptr<ArvCamera>(arv_camera_new(device_description.c_str()), [](ArvCamera* ptr) {
                            if (ptr) g_object_unref(ptr);
                        });

                    if (!ARV_IS_CAMERA(camera.get())) {
                        throw std::runtime_error(
                            fmt::format("Failed to create {} camera ({})", name, device_description));
                    }
                    else {
                        // Create a new stream object: Store as shared pointer
                        auto stream = std::shared_ptr<ArvStream>(
                            arv_camera_create_stream(camera.get(), nullptr, nullptr), [](ArvStream* ptr) {
                                if (ptr) g_object_unref(ptr);
                            });

                        if (!ARV_IS_STREAM(stream.get())) {
                            throw std::runtime_error(
                                fmt::format("Failed to create stream for {} camera ({})", name, device_description));
                        }

                        // Add camera to list.
                        it = cameras
                                 .insert(std::make_pair(serial_number,
                                                        CameraContext{
                                                            *this,
                                                            name,
                                                            0,  // fourcc is set later
                                                            num_cameras++,
                                                            Image::Lens(),      // Lens is constructed in settings
                                                            Eigen::Affine3d(),  // Hpc is set in settings
                                                            camera,
                                                            stream,
                                                            CameraContext::TimeCorrection(),
                                                        }))
                                 .first;

                        log<NUClear::INFO>(fmt::format("Connected to the {} camera ({})", name, device_description));
                    }
                }
            }

            /*****************************************************************************
             *   _   _           _       _         ____       _   _   _                  *
             *  | | | |_ __   __| | __ _| |_ ___  / ___|  ___| |_| |_(_)_ __   __ _ ___  *
             *  | | | | '_ \ / _` |/ _` | __/ _ \ \___ \ / _ \ __| __| | '_ \ / _` / __| *
             *  | |_| | |_) | (_| | (_| | ||  __/  ___) |  __/ |_| |_| | | | | (_| \__ \ *
             *   \___/| .__/ \__,_|\__,_|\__\___| |____/ \___|\__|\__|_|_| |_|\__, |___/ *
             *        |_|                                                     |___/      *
             *****************************************************************************/

            // Update settings on the camera
            auto& context    = it->second;
            const auto& name = context.name;
            auto& cam        = context.camera;
            auto& stream     = context.stream;
            auto device      = arv_camera_get_device(cam.get());

            // Stop the video stream so we can apply the settings
            arv_camera_stop_acquisition(cam.get());
            arv_stream_set_emit_signals(stream.get(), false);

            // Synchronise the clocks
            context.time = sync_clocks(device);

            // Get the fourcc code from the pixel format
            context.fourcc = description_to_fourcc(config["settings"]["PixelFormat"].as<std::string>());

            // Load Hpc from configuration
            context.Hpc = Eigen::Matrix4d(config["lens"]["Hpc"].as<Expression>());

            // Apply image offsets to lens_centre, optical axis:
            GError* error  = nullptr;
            int full_width = arv_device_get_integer_feature_value(device, "WidthMax", &error);
            if (error) {
                g_error_free(error);
                error = nullptr;
            }
            int full_height = arv_device_get_integer_feature_value(device, "HeightMax", &error);
            if (error) {
                g_error_free(error);
                error = nullptr;
            }
            log<NUClear::DEBUG>(fmt::format("Max. resolution: {} by {} on {} camera", full_width, full_height, name));

            int offset_x = config["settings"]["OffsetX"].as<Expression>();
            int offset_y = config["settings"]["OffsetY"].as<Expression>();
            int width    = config["settings"]["Width"].as<Expression>();
            int height   = config["settings"]["Height"].as<Expression>();

            // Set the lens paramters from configuration
            context.lens = Image::Lens{config["lens"]["projection"].as<std::string>(),
                                       float(config["lens"]["focal_length"].as<Expression>() * full_width / width),
                                       float(config["lens"]["fov"].as<Expression>()),
                                       ((Eigen::Vector2f(config["lens"]["centre"].as<Expression>()) * full_width)
                                        + Eigen::Vector2f(offset_x - (full_width - width - offset_x),
                                                          offset_y - (full_height - height - offset_y))
                                              * 0.5)
                                           / width};

            log<NUClear::DEBUG>(fmt::format("Lens Centre: [{} , {}] on {} camera",
                                            config["lens"]["centre"][0].as<double>(),
                                            config["lens"]["centre"][1].as<double>(),
                                            name));
            log<NUClear::DEBUG>(fmt::format(
                "Lens Centre: [{} , {}] on {} camera", context.lens.centre[0], context.lens.centre[1], name));


            // If the lens fov was auto we need to correct it
            if (!std::isfinite(context.lens.fov)) {
                double a = height / width;
                std::array<double, 4> options{
                    utility::vision::unproject(Eigen::Vector2f(0, 0), context.lens, Eigen::Vector2f(1, a)).x(),
                    utility::vision::unproject(Eigen::Vector2f(1, 0), context.lens, Eigen::Vector2f(1, a)).x(),
                    utility::vision::unproject(Eigen::Vector2f(0, a), context.lens, Eigen::Vector2f(1, a)).x(),
                    utility::vision::unproject(Eigen::Vector2f(1, a), context.lens, Eigen::Vector2f(1, a)).x()};

                context.lens.fov = std::acos(*std::min_element(options.begin(), options.end())) * 2.0;
                log<NUClear::DEBUG>(fmt::format("Calculated fov for {} camera as {}", name, context.lens.fov));
            }

            // Apply the region to the camera
            arv_camera_set_region(cam.get(), offset_x, offset_y, width, height);

            // Go through our settings and apply them to the camera
            for (const auto& cfg : config["settings"].config) {
                std::string key = cfg.first.as<std::string>();

                // Skip the region keys as we handle them above
                if (key == "Width" || key == "Height" || key == "OffsetX" || key == "OffsetY") {
                    continue;
                };

                // Get the feature node
                auto feature = arv_device_get_feature(device, key.c_str());
                try {
                    std::string message;
                    if (feature == nullptr) {
                        // Doesn't have this feature
                        NUClear::log<NUClear::ERROR>(
                            fmt::format("The {} camera does not have a setting named {}", name, key));
                    }

                    // Integer setting
                    else if (ARV_IS_GC_INTEGER_NODE(feature)) {
                        auto setting  = reinterpret_cast<ArvGcInteger*>(feature);
                        int64_t value = std::llround(double(cfg.second.as<Expression>()));
                        message       = set_setting(setting, value);
                    }

                    // Floating point setting
                    else if (ARV_IS_GC_FLOAT_NODE(feature)) {
                        auto setting = reinterpret_cast<ArvGcFloat*>(feature);
                        double value = cfg.second.as<Expression>();
                        message      = set_setting(setting, value);
                    }

                    // Boolean setting
                    else if (ARV_IS_GC_BOOLEAN(feature)) {
                        auto setting = reinterpret_cast<ArvGcBoolean*>(feature);
                        bool value   = bool(cfg.second.as<Expression>());
                        message      = set_setting(setting, value);
                    }

                    // Enumeration setting
                    else if (ARV_IS_GC_ENUMERATION(feature)) {
                        auto setting      = reinterpret_cast<ArvGcEnumeration*>(feature);
                        std::string value = cfg.second.as<std::string>();
                        message           = set_setting(setting, value);
                    }
                    else {
                        log<NUClear::ERROR>(fmt::format("The type of setting {} is not currently handled", key));
                    }

                    // If we had a message pass it onto the user
                    if (!message.empty()) {
                        log<NUClear::INFO>(fmt::format("Updated {} on {} camera: {}", key, name, message));
                    }
                }
                catch (const std::runtime_error& e) {
                    log<NUClear::ERROR>(fmt::format("Error setting {} on {} camera: {}", key, name, e.what()));
                }
            }

            // Check for gigevision device and set packet size to jumbo packets
            if (arv_camera_is_gv_device(cam.get())) {
                arv_camera_gv_set_packet_size(cam.get(), 8192);
                g_object_set(stream.get(), "packet-resend", ARV_GV_STREAM_PACKET_RESEND_NEVER, nullptr);
            }

            // Add buffers to the queue
            int payload_size = arv_camera_get_payload(cam.get());
            for (size_t i = 0; i < config["buffer_count"].as<size_t>(); i++) {
                // TODO(trent) Eventually we should use preallocated page aligned data so that we can map directly to
                // the GPU.
                // TODO(trent) Make an std::vector and use it in the buffer to avoid copying to one later
                arv_stream_push_buffer(stream.get(), arv_buffer_new(payload_size, nullptr));
            }

            // Connect signal events
            g_signal_connect(stream.get(), "new-buffer", G_CALLBACK(&Camera::emit_image), &it->second);
            g_signal_connect(
                arv_camera_get_device(cam.get()), "control-lost", G_CALLBACK(&Camera::control_lost), &it->second);
            // Start aquisition
            arv_camera_start_acquisition(cam.get());
            arv_stream_set_emit_signals(stream.get(), true);
        });

        on<Trigger<Sensors>>().then("Buffer Sensors", [this](const Sensors& sensors) {
            std::lock_guard<std::mutex> lock(sensors_mutex);
            auto now = NUClear::clock::now();
            Hpws.resize(std::distance(Hpws.begin(), std::remove_if(Hpws.begin(), Hpws.end(), [now](const auto& v) {
                                          return v.first < (now - std::chrono::milliseconds(500));
                                      })));

            // Get torso to head, and torso to world
            Eigen::Affine3d Htp(sensors.forward_kinematics[ServoID::HEAD_PITCH]);
            Eigen::Affine3d Htw(sensors.Htw);
            Eigen::Affine3d Hpw = Htp.inverse() * Htw;

            Hpws.push_back(std::make_pair(sensors.timestamp, Hpw));
        });

        on<Shutdown>().then([this] {
            for (auto& camera : cameras) {
                // Stop the video stream.
                arv_camera_stop_acquisition(camera.second.camera.get());
                arv_stream_set_emit_signals(camera.second.stream.get(), false);
            }
            arv_shutdown();
            cameras.clear();
        });
    }

    void Camera::emit_image(ArvStream* stream, CameraContext* context) {
        using namespace std::chrono;
        // Get now here, as close as possible to when we pop the buffer
        int64_t now       = duration_cast<nanoseconds>(NUClear::clock::now().time_since_epoch()).count();
        ArvBuffer* buffer = arv_stream_try_pop_buffer(stream);
        Camera& reactor   = context->reactor;

        if (buffer != nullptr) {
            if (arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS) {
                int width, height;
                size_t buffSize;
                arv_buffer_get_image_region(buffer, nullptr, nullptr, &width, &height);
                const uint8_t* buff = reinterpret_cast<const uint8_t*>(arv_buffer_get_data(buffer, &buffSize));

                auto& timesync = context->time;

                // Get the timestamp from the buffer
                int64_t ts = arv_buffer_get_timestamp(buffer);

                // If we couldn't calculate the timestamp offset using the cameras API, we have to do it on the fly
                // and hope that our answer isn't too far off the truth
                if (timesync.live) {
                    // Offset in nanoseconds and seconds
                    int64_t o_ns = (now - ts);

                    // Initialise the filter if this is the first time
                    timesync.offset = timesync.offset == 0 ? o_ns : timesync.offset;

                    // Add process noise
                    timesync.kf.p += timesync.kf.q;

                    // Measurement update
                    double k        = timesync.kf.p / (timesync.kf.p + timesync.kf.r);
                    timesync.offset = std::llround(timesync.offset + k * (o_ns - timesync.offset));
                    timesync.kf.p   = (1.0 - k) * timesync.kf.p;
                }
                else {
                    // Work out the total amount of time since calibration for both local and remote clocks
                    int64_t total_cam   = ts - timesync.drift.cam_at_calibration;
                    int64_t total_local = now - timesync.drift.local_at_calibration;

                    // Until an appropriate amount of time has passed use the offset to work out what a bad clock
                    // drift would be using exponential smoothing
                    if (total_local < TRANSFER_OFFSET_OBSERVE_TIME) {
                        timesync.drift.max_clock_drift = timesync.drift.max_clock_drift == 0
                                                             ? std::abs(total_cam - total_local) + MAX_CLOCK_DRIFT
                                                             : (timesync.drift.max_clock_drift * 9
                                                                + std::abs(total_cam - total_local) + MAX_CLOCK_DRIFT)
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
                                std::abs(total_cam - total_local) / 1e6,
                                timesync.drift.max_clock_drift / 1e6));

                            arv_camera_stop_acquisition(context->camera.get());
                            arv_stream_set_emit_signals(context->stream.get(), false);
                            context->time = sync_clocks(arv_camera_get_device(context->camera.get()));
                            arv_stream_set_emit_signals(context->stream.get(), true);
                            arv_camera_start_acquisition(context->camera.get());
                        }
                    }
                }

                // Apply our timestamp offset
                ts = ts + context->time.offset;

                auto msg        = std::make_unique<Image>();
                msg->format     = context->fourcc;
                msg->dimensions = Eigen::Matrix<unsigned int, 2, 1>(width, height);
                // TODO(trent) use an std::vector here to avoid the copy
                msg->data.insert(msg->data.end(), buff, buff + buffSize);
                msg->camera_id = context->camera_id;
                msg->name      = context->name;
                msg->timestamp = NUClear::clock::time_point(nanoseconds(ts));

                Eigen::Affine3d Hcw;

                /* Mutex Scope */ {
                    std::lock_guard<std::mutex> lock(reactor.sensors_mutex);

                    Eigen::Affine3d Hpc = context->Hpc;
                    Eigen::Affine3d Hpw;
                    if (reactor.Hpws.empty()) {
                        Hpw = Eigen::Affine3d::Identity();
                    }
                    else {
                        // Find the first time that is not less than the target time
                        auto Hpw_it = std::lower_bound(reactor.Hpws.begin(),
                                                       reactor.Hpws.end(),
                                                       std::make_pair(msg->timestamp, Eigen::Affine3d::Identity()),
                                                       [](const auto& a, const auto& b) { return a.first < b.first; });

                        if (Hpw_it == reactor.Hpws.end()) {
                            // Image is newer than most recent sensors
                            Hpw = std::prev(Hpw_it)->second;
                        }
                        else if (Hpw_it == reactor.Hpws.begin()) {
                            // Image is older than oldest sensors
                            Hpw = Hpw_it->second;
                        }
                        else {
                            // Check Hpw_it and std::prev(Hpw) for closest match
                            Hpw = std::abs((Hpw_it->first - msg->timestamp).count())
                                          < std::abs((std::prev(Hpw_it)->first - msg->timestamp).count())
                                      ? Hpw_it->second
                                      : std::prev(Hpw_it)->second;
                        }
                    }

                    Hcw = Hpc.inverse() * Hpw;  // Transform: From world to platform to camera to image.
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
                NUClear::log<NUClear::WARN>("Failed getting buffer from", context->name, "with error", msg);
            }
        }
    }

    void Camera::control_lost(ArvGvDevice*, CameraContext* context) {
        NUClear::log<NUClear::FATAL>(fmt::format("Control of a the {} camera has been lost", context->name));
    }

}  // namespace input
}  // namespace module
