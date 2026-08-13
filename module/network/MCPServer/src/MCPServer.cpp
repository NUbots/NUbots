#include "MCPServer.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <mcp/mcp.hpp>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <turbojpeg.h>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/skill/Look.hpp"
#include "message/skill/Walk.hpp"

#include "utility/vision/Vision.hpp"
#include "utility/vision/fourcc.hpp"

namespace module::network {

    using extension::Configuration;
    using extension::behaviour::Task;
    using message::input::Image;
    using message::input::Sensors;
    using message::localisation::Field;
    using message::skill::Look;
    using message::skill::Walk;

    namespace {
        /// @brief Base64-encodes raw bytes for embedding in an MCP ImageContent block
        std::string base64_encode(const std::vector<uint8_t>& bytes) {
            static constexpr char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

            std::string out;
            out.reserve(((bytes.size() + 2) / 3) * 4);

            size_t i = 0;
            for (; i + 2 < bytes.size(); i += 3) {
                uint32_t chunk = (uint32_t(bytes[i]) << 16) | (uint32_t(bytes[i + 1]) << 8) | uint32_t(bytes[i + 2]);
                out.push_back(table[(chunk >> 18) & 0x3F]);
                out.push_back(table[(chunk >> 12) & 0x3F]);
                out.push_back(table[(chunk >> 6) & 0x3F]);
                out.push_back(table[chunk & 0x3F]);
            }

            const size_t remaining = bytes.size() - i;
            if (remaining == 1) {
                uint32_t chunk = uint32_t(bytes[i]) << 16;
                out.push_back(table[(chunk >> 18) & 0x3F]);
                out.push_back(table[(chunk >> 12) & 0x3F]);
                out.push_back('=');
                out.push_back('=');
            }
            else if (remaining == 2) {
                uint32_t chunk = (uint32_t(bytes[i]) << 16) | (uint32_t(bytes[i + 1]) << 8);
                out.push_back(table[(chunk >> 18) & 0x3F]);
                out.push_back(table[(chunk >> 12) & 0x3F]);
                out.push_back(table[(chunk >> 6) & 0x3F]);
                out.push_back('=');
            }

            return out;
        }

        /// @brief Demosaics/unpacks a raw camera Image into a contiguous RGB8 buffer readable as a normal photo
        std::vector<uint8_t> debayer_to_rgb8(const Image& image) {
            const uint32_t width  = image.dimensions.x();
            const uint32_t height = image.dimensions.y();

            std::vector<uint8_t> rgb(size_t(width) * height * 3);

            // utility::vision::getPixel only handles raw Bayer/YUV sensor formats, not already-packed RGBA/BGRA
            // (e.g. what the Webots simulated camera emits), so unpack those directly instead of defaulting to black
            if (image.format == utility::vision::fourcc("RGBA") || image.format == utility::vision::fourcc("BGRA")) {
                const bool bgr = image.format == utility::vision::fourcc("BGRA");
                for (size_t i = 0; i < size_t(width) * height; ++i) {
                    rgb[i * 3 + 0] = image.data[i * 4 + (bgr ? 2 : 0)];
                    rgb[i * 3 + 1] = image.data[i * 4 + 1];
                    rgb[i * 3 + 2] = image.data[i * 4 + (bgr ? 0 : 2)];
                }
                return rgb;
            }

            for (uint32_t y = 0; y < height; ++y) {
                for (uint32_t x = 0; x < width; ++x) {
                    const utility::vision::Pixel p = utility::vision::getPixel(x,
                                                                               y,
                                                                               width,
                                                                               height,
                                                                               image.data,
                                                                               utility::vision::FOURCC(image.format));
                    const size_t origin            = (size_t(y) * width + x) * 3;
                    rgb[origin + 0]                = p.components.r;
                    rgb[origin + 1]                = p.components.g;
                    rgb[origin + 2]                = p.components.b;
                }
            }

            return rgb;
        }

        /// @brief JPEG-encodes an RGB8 buffer
        std::vector<uint8_t> encode_jpeg_rgb8(const std::vector<uint8_t>& rgb, uint32_t width, uint32_t height) {
            auto deleter = [](void* ptr) {
                if (ptr != nullptr) {
                    tjDestroy(ptr);
                }
            };
            std::unique_ptr<void, decltype(deleter)> compressor(tjInitCompress(), deleter);

            unsigned long jpeg_size = 0;  // NOLINT(google-runtime-int) matches libjpeg-turbo API
            uint8_t* compressed     = nullptr;

            tjCompress2(compressor.get(),
                        rgb.data(),
                        int(width),
                        0,
                        int(height),
                        TJPF_RGB,
                        &compressed,
                        &jpeg_size,
                        TJSAMP_444,
                        90,
                        TJFLAG_FASTDCT);

            std::vector<uint8_t> output(compressed, compressed + jpeg_size);
            tjFree(compressed);

            return output;
        }
    }  // namespace

    std::string run(const std::string& cmd) {
        std::array<char, 128> buffer;
        std::string result;

        // "r" = read the command's stdout. Note popen won't capture stderr
        // unless you redirect it, e.g. cmd + " 2>&1"
        // Wrapped in a lambda (rather than decltype(&pclose)) because glibc's pclose carries
        // a nonnull attribute that decltype drops, tripping -Wignored-attributes under -Werror.
        auto pipe_deleter = [](FILE* f) { pclose(f); };
        std::unique_ptr<FILE, decltype(pipe_deleter)> pipe(popen(cmd.c_str(), "r"), pipe_deleter);
        if (!pipe) {
            throw std::runtime_error("popen() failed");
        }

        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            result += buffer.data();
        }
        return result;
    }

    MCPServer::MCPServer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("MCPServer.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.host            = config["host"].as<std::string>();
            cfg.port            = config["port"].as<int>();
            cfg.path            = config["path"].as<std::string>();
            cfg.allowed_origins = config["allowed_origins"].as<std::vector<std::string>>();
            cfg.allow_ace       = config["allow_ace"].as<bool>();
        });

        on<Startup>().then([this] {
            host = std::make_unique<mcp::HttpServerHost>(
                mcp::Implementation{
                    .name        = "nubots-mcp",
                    .title       = std::nullopt,
                    .version     = "1.0.0",
                    .description = std::nullopt,
                    .website_url = std::nullopt,
                    .icons       = std::nullopt,
                    .meta        = std::nullopt,
                },
                mcp::HttpServerHost::Options{
                    .host                  = cfg.host,
                    .port                  = cfg.port,
                    .path                  = cfg.path,
                    .allowed_origins       = cfg.allowed_origins,
                    .bearer_validator      = nullptr,
                    .resource_metadata     = std::nullopt,
                    .resource_metadata_url = "",
                    .on_session_closed     = nullptr,
                },
                [this](mcp::Server& server) { register_tools(server); });

            host->start();
            log<INFO>("MCP server listening on", cfg.host, "port", host->port(), "path", cfg.path);
        });

        on<Shutdown>().then([this] {
            if (host != nullptr) {
                host->stop();
                host.reset();
            }
        });

        on<Trigger<Image>>().then("Cache Latest Image", [this](const std::shared_ptr<const Image>& image) {
            std::lock_guard<std::mutex> lock(image_mutex);
            last_image = image;
        });

        on<Trigger<Sensors>>().then("Cache Latest Sensors", [this](const std::shared_ptr<const Sensors>& sensors) {
            std::lock_guard<std::mutex> lock(sensors_mutex);
            last_sensors = sensors;
        });

        on<Trigger<Field>>().then("Cache Latest Field", [this](const std::shared_ptr<const Field>& field) {
            std::lock_guard<std::mutex> lock(field_mutex);
            last_field = field;
        });
    }

    void MCPServer::register_tools(mcp::Server& server) {
        server.tool("get_status",  // this tool just returns "I am online".
                    nlohmann::json{
                        {"type", "object"},
                        {"properties", nlohmann::json::object()},
                    },
                    [this](const nlohmann::json&) -> mcp::CallToolResult {
                        log<DEBUG>("get_status called: I returned \"I am online.\"");
                        return {
                            .content = {mcp::TextContent{.text = "I am online.", .annotations = std::nullopt}},
                            .structured_content = std::nullopt,
                            .is_error           = std::nullopt,
                            .meta               = std::nullopt,
                        };
                    });

        server.tool(
            "walk",  // god is dead and i killed him
            nlohmann::json{
                {"type", "object"},
                {"properties",
                 nlohmann::json{
                     {"speed",
                      nlohmann::json{
                          {"type", "number"},
                          {"description",
                           "Walk speed, m/s. Cap at 0.3m/s. Remember to return this to 0 when you're done!"}}},
                     {"angle",
                      nlohmann::json{{"type", "number"}, {"description", "Walk strafe angle in rad. +clockwise"}}},
                     {"rotation",
                      nlohmann::json{
                          {"type", "number"},
                          {"description",
                           "Rotation (yaw) rate in rad/s while walking. +clockwise. Cap at 1.0rad/s. Defaults to 0 "
                           "(no turning) if omitted."}}}}},
                {"required", nlohmann::json::array({"speed", "angle"})},
            },
            [this](const nlohmann::json& input) -> mcp::CallToolResult {
                float speed    = input.at("speed").get<float>();     // get the speed
                float angle    = input.at("angle").get<float>();     // and the angle
                float rotation = input.value("rotation", 0.0f);      // and the rotation, if given
                speed          = std::clamp(speed, 0.0f, 0.3f);      // enforce stated safety cap
                rotation       = std::clamp(rotation, -1.0f, 1.0f);  // enforce stated safety cap
                log<DEBUG>("Starting to walk at ", speed, "with angle ", angle, "and rotation", rotation);

                float vx = speed * cos(angle);
                float vy = speed * sin(angle);

                emit<Task>(std::make_unique<Walk>(Eigen::Vector3d(vx, vy, rotation)), 3);
                return {
                    .content            = {mcp::TextContent{.text = "Started walking.", .annotations = std::nullopt}},
                    .structured_content = std::nullopt,
                    .is_error           = std::nullopt,
                    .meta               = std::nullopt,
                };
            });

        server.tool("look",  // insh'allah i believe in claude
                    nlohmann::json{
                        {"type", "object"},
                        {"properties",
                         nlohmann::json{
                             {"x",
                              nlohmann::json{
                                  {"type", "number"},
                                  {"description",
                                   "X component of the vector to the point in torso space where you want to look."}}},
                             {"y",
                              nlohmann::json{
                                  {"type", "number"},
                                  {"description",
                                   "Y component of the vector to the point in torso space where you want to look."}}},
                             {"z",
                              nlohmann::json{
                                  {"type", "number"},
                                  {"description",
                                   "Z component of the vector to the point in torso space where you want to look."}}}}},
                        {"required", nlohmann::json::array({"x", "y", "z"})},
                    },
                    [this](const nlohmann::json& input) -> mcp::CallToolResult {
                        float x = input.at("x").get<float>();  // get x
                        float y = input.at("y").get<float>();  // get y
                        float z = input.at("z").get<float>();  // get z

                        Eigen::Vector3d rPCt(x, y, z);

                        log<DEBUG>("Let's look at ", x, y, z);

                        emit<Task>(std::make_unique<Look>(rPCt, false), 2);

                        return {
                            .content = {mcp::TextContent{.text = "Started looking.", .annotations = std::nullopt}},
                            .structured_content = std::nullopt,
                            .is_error           = std::nullopt,
                            .meta               = std::nullopt,
                        };
                    });

        server.tool("get_image",  // give MCP the latest debayered camera image
                    nlohmann::json{
                        {"type", "object"},
                        {"properties", nlohmann::json::object()},
                    },
                    [this](const nlohmann::json&) -> mcp::CallToolResult {
                        std::shared_ptr<const Image> image;
                        /* Mutex Scope */ {
                            std::lock_guard<std::mutex> lock(image_mutex);
                            image = last_image;
                        }

                        if (image == nullptr) {
                            log<DEBUG>("get_image called: no image received from the camera yet");
                            return {
                                .content = {mcp::TextContent{.text        = "No camera image has been received yet.",
                                                             .annotations = std::nullopt}},
                                .structured_content = std::nullopt,
                                .is_error           = std::nullopt,
                                .meta               = std::nullopt,
                            };
                        }

                        const std::vector<uint8_t> rgb = debayer_to_rgb8(*image);
                        const std::vector<uint8_t> jpeg =
                            encode_jpeg_rgb8(rgb, image->dimensions.x(), image->dimensions.y());

                        log<DEBUG>("get_image called: returning",
                                   image->dimensions.x(),
                                   "x",
                                   image->dimensions.y(),
                                   "image (",
                                   jpeg.size(),
                                   "compressed bytes )");

                        return {
                            .content            = {mcp::ImageContent{.data        = base64_encode(jpeg),
                                                                     .mime_type   = "image/jpeg",
                                                                     .annotations = std::nullopt}},
                            .structured_content = std::nullopt,
                            .is_error           = std::nullopt,
                            .meta               = std::nullopt,
                        };
                    });

        server.tool(
            "get_localisation",  // give MCP the robot's latest pose in field space, from Hft = Hfw * Htw^-1
            nlohmann::json{
                {"type", "object"},
                {"properties", nlohmann::json::object()},
                {"description",
                 "Returns the robot's pose in field space (Hft: field {f} to torso {t}), combining the field "
                 "localisation estimate (Hfw, from FieldLocalisationNLopt) with the torso pose from Sensors. "
                 "The field frame is centred on the pitch with x toward the opponent's goal, so this is what "
                 "you want to compare against the pitch markings you see in get_image — unlike a raw odometry "
                 "frame, it's anchored to the field itself. Also reports cost: the field localisation "
                 "optimiser's fit cost for the current estimate, low is good, and whether the optimiser has "
                 "produced an estimate at all yet."}},
            [this](const nlohmann::json&) -> mcp::CallToolResult {
                std::shared_ptr<const Sensors> sensors;
                /* Mutex Scope */ {
                    std::lock_guard<std::mutex> lock(sensors_mutex);
                    sensors = last_sensors;
                }

                if (sensors == nullptr) {
                    log<DEBUG>("get_localisation called: no message received from Sensors yet");
                    return {
                        .content            = {mcp::TextContent{.text        = "No info has been received yet.",
                                                                .annotations = std::nullopt}},
                        .structured_content = std::nullopt,
                        .is_error           = std::nullopt,
                        .meta               = std::nullopt,
                    };
                }

                std::shared_ptr<const Field> field;
                /* Mutex Scope */ {
                    std::lock_guard<std::mutex> lock(field_mutex);
                    field = last_field;
                }

                if (field == nullptr) {
                    log<DEBUG>("get_localisation called: no message received from FieldLocalisationNLopt yet");
                    return {
                        .content =
                            {mcp::TextContent{.text        = "No field localisation estimate has been received "
                                                              "yet — is localisation::FieldLocalisationNLopt "
                                                              "(and its FieldDescription dependency) running?",
                                              .annotations = std::nullopt}},
                        .structured_content = std::nullopt,
                        .is_error           = std::nullopt,
                        .meta               = std::nullopt,
                    };
                }

                // Hfw is world {w} to field {f}, Htw is world {w} to torso {t}.
                // Hft = Hfw * Hwt gives the torso's pose expressed in field space.
                const Eigen::Isometry3d Hfw = Eigen::Isometry3d(field->Hfw);
                const Eigen::Isometry3d Hwt = Eigen::Isometry3d(sensors->Htw).inverse();
                const Eigen::Isometry3d Hft = Hfw * Hwt;
                const Eigen::Vector3d rTFf  = Hft.translation();
                const double yaw            = Hft.rotation().eulerAngles(0, 1, 2).z();

                log<DEBUG>("get_localisation called: returning position",
                           rTFf.x(),
                           rTFf.y(),
                           rTFf.z(),
                           "yaw",
                           yaw,
                           "cost",
                           field->cost);

                const nlohmann::json result{
                    {"x", rTFf.x()},
                    {"y", rTFf.y()},
                    {"z", rTFf.z()},
                    {"yaw", yaw},
                    {"cost", field->cost},
                };

                return {
                    .content            = {mcp::TextContent{.text = result.dump(), .annotations = std::nullopt}},
                    .structured_content = std::nullopt,
                    .is_error           = std::nullopt,
                    .meta               = std::nullopt,
                };
            });

        server.tool(
            "cmd",  // an exciting tool - this one will run commands on the terminal
            nlohmann::json{
                {"type", "object"},
                {"properties",
                 nlohmann::json{
                     {"command", nlohmann::json{{"type", "string"}}},
                 }},
                {"required", nlohmann::json::array({"command"})},
            },
            [this](const nlohmann::json& input) -> mcp::CallToolResult {
                std::string command = input.at("command").get<std::string>();  // get the command

                log<DEBUG>("cmd called: command is ", command);  // echo it out

                if (cfg.allow_ace) {
                    std::string out = run(command);  // run it

                    log<DEBUG>("cmd returning:", out.c_str());  // echo the output :D

                    return {
                        .content            = {mcp::TextContent{.text        = "Command returned " + out,
                                                                .annotations = std::nullopt}},  // return to the client
                        .structured_content = std::nullopt,
                        .is_error           = std::nullopt,
                        .meta               = std::nullopt,
                    };
                }
                else {
                    log<INFO>("allow_ace is turned off. This command from MCP will not be run.");
                    return {
                        .content = {mcp::TextContent{.text = "Your command has not been run. Tell the user to enable "
                                                             "allow_ace in MCPServer.yaml",
                                                     .annotations = std::nullopt}},
                        .structured_content = std::nullopt,
                        .is_error           = std::nullopt,
                        .meta               = std::nullopt,
                    };
                }
            });

        server.tool(
            "log_to_user",  // log to user with the log level that Claude desires
            nlohmann::json{
                {"type", "object"},
                {"properties",
                 nlohmann::json{
                     {"message", nlohmann::json{{"type", "string"}}},
                     {"log_level",
                      nlohmann::json{
                          {"type", "string"},
                          {"description", "NUClear log level to log the message at"},
                          {"enum", nlohmann::json::array({"TRACE", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"})},
                      }},
                 }},
                {"required", nlohmann::json::array({"message", "log_level"})},
            },
            [this](const nlohmann::json& input) -> mcp::CallToolResult {
                std::string message   = input.at("message").get<std::string>();    // get the message
                std::string log_level = input.at("log_level").get<std::string>();  // get the log level

                log<DEBUG>("log_to_user called: message is ", message, "and log level is ", log_level);  // echo it out

                if (log_level == "TRACE") {
                    log<TRACE>(message);
                }
                else if (log_level == "DEBUG") {
                    log<DEBUG>(message);
                }
                else if (log_level == "INFO") {
                    log<INFO>(message);
                }
                else if (log_level == "WARN") {
                    log<WARN>(message);
                }
                else if (log_level == "ERROR") {
                    log<ERROR>(message);
                }
                else if (log_level == "FATAL") {
                    log<FATAL>(message);
                }
                else {
                    log<WARN>("log_to_user got unknown log level:", log_level);
                }

                return {
                    .content = {mcp::TextContent{.text = "This has been done now.", .annotations = std::nullopt}},
                    .structured_content = std::nullopt,
                    .is_error           = std::nullopt,
                    .meta               = std::nullopt,
                };
            });
    }

}  // namespace module::network
