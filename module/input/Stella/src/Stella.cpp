#include "Stella.hpp"

#include <filesystem>
#include <opencv2/opencv.hpp>
#include <stella_vslam/publish/frame_publisher.h>
#include "socket_publisher/publisher.h"

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
#include "message/output/CompressedImage.hpp"

#include "utility/vision/Vision.hpp"
#include "utility/vision/fourcc.hpp"

namespace fs = std::filesystem;
namespace module::input {

    using extension::Configuration;
    using message::input::Image;
    using message::output::CompressedImage;

    void Stella::start_socket_once() {
        if (socket_thread_running.exchange(true)) return;
        socket_thread = std::thread([this] {
            // blocking loop; exits after request_terminate()
            socket_publisher->run();
        });
    }

    void Stella::stop_socket_and_publishers() {
        if (!socket_thread_running) return;
        // Stop socket first (it’s the consumer of frame/map pubs)
        socket_publisher->request_terminate();

        if (socket_thread.joinable()) socket_thread.join();
        socket_thread_running = false;
    }

    Stella::Stella(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        on<Configuration>("Stella.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            if (slam_system) {
                log<INFO>("SLAM system already initialized, shutting down gracefully...");
                stop_socket_and_publishers();
                slam_system->shutdown();
                slam_system.reset();
                frame_publisher.reset();
                map_publisher.reset();
                socket_publisher.reset();
            }

            std::string config_path = config["config_path"].as<std::string>();
            std::string vocab_file  = config["vocab_path"].as<std::string>();

            slam_config = std::make_shared<stella_vslam::config>(config_path);
            slam_system = std::make_shared<stella_vslam::system>(slam_config, vocab_file);
            slam_system->startup();

            // 🔹 Get the frame publisher for debug visualization
            frame_publisher = slam_system->get_frame_publisher();

            // get map publisher
            map_publisher = slam_system->get_map_publisher();

            socket_publisher = std::make_shared<socket_publisher::publisher>(
                config["SocketPublisher"],
                slam_system,
                frame_publisher,
                map_publisher
            );

            start_socket_once();

            log<INFO>("Stella initialized. Debug frames will be saved to stella_output/");
        });

        on<Trigger<Image>, Single>().then([this](const Image& image) {
            int width  = image.dimensions.x();
            int height = image.dimensions.y();

            cv::Mat img_cv;
            switch (image.format) {
                case utility::vision::fourcc("BGR3"):
                    img_cv = cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(image.data.data()));
                    break;
                case utility::vision::fourcc("RGBA"):
                    img_cv = cv::Mat(height, width, CV_8UC4, const_cast<uint8_t*>(image.data.data()));
                    cv::cvtColor(img_cv, img_cv, cv::COLOR_RGBA2BGR);
                    break;
                case utility::vision::FOURCC::RGGB:
                    img_cv = cv::Mat(height, width, CV_8UC1, const_cast<uint8_t*>(image.data.data()));
                    cv::cvtColor(img_cv, img_cv, cv::COLOR_BayerRG2BGR);
                    break;
                default: log<WARN>("Unsupported image format: ", utility::vision::fourcc(image.format)); return;
            }

            // Convert to grayscale for SLAM
            cv::Mat img_rgb;
            // RGBA to RGB
            cv::cvtColor(img_cv, img_rgb, cv::COLOR_RGBA2BGR);

            double timestamp = NUClear::clock::now().time_since_epoch().count() / 1e9;
            slam_system->feed_monocular_frame(img_rgb, timestamp);

            // 🔹 Get Stella debug frame with ORB detections and emit as CompressedImage
            cv::Mat debug_frame = frame_publisher->draw_frame();
            if (!debug_frame.empty()) {
                // Convert BGR to RGB format for JPEG compression
                cv::Mat rgb_frame;
                cv::cvtColor(debug_frame, rgb_frame, cv::COLOR_BGR2RGB);

                // Compress to JPEG using OpenCV
                std::vector<uint8_t> jpeg_data;
                std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 85}; // 85% quality
                cv::imencode(".jpg", rgb_frame, jpeg_data, compression_params);

                // Create CompressedImage message
                auto debug_image = std::make_unique<CompressedImage>();

                // Set basic fields
                debug_image->format = utility::vision::fourcc("JPEG");
                debug_image->dimensions = Eigen::Vector2<unsigned int>(rgb_frame.cols, rgb_frame.rows);
                debug_image->id = 1;
                debug_image->name = "stella_debug_frame";
                debug_image->timestamp = image.timestamp;  // Use the same timestamp as input image
                debug_image->Hcw = image.Hcw;  // Copy the camera transform
                debug_image->lens.projection = static_cast<int>(image.lens.projection);
                debug_image->lens.focal_length = image.lens.focal_length;
                debug_image->lens.fov = image.lens.fov;
                debug_image->lens.centre = image.lens.centre;
                debug_image->lens.k = image.lens.k;

                // Copy compressed JPEG data
                debug_image->data = jpeg_data;

                // Emit the compressed debug image
                emit(std::move(debug_image));

                if (log_level <= DEBUG) {
                    log<DEBUG>("Emitted Stella debug frame with ORB detections, frame: ", debug_image->id,
                            ", size: ", jpeg_data.size(), " bytes");
                }
            }
        });
    }

}  // namespace module::input
