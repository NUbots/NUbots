#include "Stella.hpp"

#include <filesystem>
#include <opencv2/opencv.hpp>
#include <stella_vslam/publish/frame_publisher.h>

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
#include "message/output/CompressedImage.hpp"
#include "message/input/Sensors.hpp"

#include "utility/vision/Vision.hpp"
#include "utility/vision/fourcc.hpp"
#include "utility/math/euler.hpp"
#include <thread>


namespace fs = std::filesystem;
namespace module::input {

    using extension::Configuration;
    using message::input::Image;
    using message::output::CompressedImage;
    using message::input::Sensors;

    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;

    Stella::Stella(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        on<Configuration>("Stella.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            if (slam_system) {
                log<INFO>("SLAM system already initialized, shutting down gracefully...");
                slam_system->shutdown();
                slam_system.reset();
                frame_publisher.reset();
            }

            std::string config_path = config["config_path"].as<std::string>();
            std::string vocab_file  = config["vocab_path"].as<std::string>();
            auto socket_config = config.config["SocketPublisher"];

            slam_config = std::make_shared<stella_vslam::config>(config_path);
            slam_system = std::make_shared<stella_vslam::system>(slam_config, vocab_file);
            slam_system->startup();

            // 🔹 Get the frame publisher for debug visualization
            frame_publisher = slam_system->get_frame_publisher();

            map_publisher = slam_system->get_map_publisher();

            log<INFO>("Creating socket publisher");
            publisher = std::make_shared<socket_publisher::publisher>(
                socket_config,
                slam_system,
                frame_publisher,
                map_publisher);

            log<INFO>("Stella initialized. Debug frames will be saved to stella_output/");
        });

        on<Trigger<Image>, Single, MainThread>().then([this](const Image& image) {
            // if (publisher)
            // {
            // publisher->run();  // This blocks this thread, not the main thread
            // }


            int width  = image.dimensions.x();
            int height = image.dimensions.y();

            cv::Mat img_cv;
            // Convert to grayscale for SLAM
            cv::Mat img_rgb;
            switch (image.format) {
                case utility::vision::fourcc("BGR3"):
                    img_cv = cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(image.data.data()));
                    break;
                case utility::vision::fourcc("RGBA"):
                    img_cv = cv::Mat(height, width, CV_8UC4, const_cast<uint8_t*>(image.data.data()));
                    cv::cvtColor(img_cv, img_rgb, cv::COLOR_RGBA2RGB);
                    break;
                case utility::vision::FOURCC::RGGB:
                    img_cv = cv::Mat(height, width, CV_8UC1, const_cast<uint8_t*>(image.data.data()));
                    cv::cvtColor(img_cv, img_rgb, cv::COLOR_BayerRG2RGB);
                    break;
                default: log<WARN>("Unsupported image format: ", utility::vision::fourcc(image.format)); return;
            }


            // RGBA to RGB
            double timestamp = NUClear::clock::now().time_since_epoch().count() / 1e9;
            auto camera_pose = slam_system->feed_monocular_frame(img_rgb, timestamp);

            // 🔹 Get Stella debug frame with ORB detections and emit as CompressedImage
            cv::Mat debug_frame = frame_publisher->draw_frame();


            if (!debug_frame.empty()) {
                // // Convert BGR to RGB format for JPEG compression
                // cv::Mat rgb_frame;
                // if (image.format == utility::vision::FOURCC::RGGB) {
                //     cv::cvtColor(debug_frame, rgb_frame, cv::COLOR_BayerRG2RGB);
                // } else {
                //     cv::cvtColor(debug_frame, rgb_frame, cv::COLOR_BGR2RGB);
                // }

                // Compress to JPEG using OpenCV
                std::vector<uint8_t> jpeg_data;
                std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 85}; // 85% quality
                cv::imencode(".jpg", debug_frame, jpeg_data, compression_params);

                // Create CompressedImage message
                auto debug_image = std::make_unique<CompressedImage>();

                // Set basic fields
                debug_image->format = utility::vision::fourcc("JPEG");
                debug_image->dimensions = Eigen::Vector2<unsigned int>(debug_frame.cols, debug_frame.rows);
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

            // Emit a sensors message
            auto sensors = std::make_unique<Sensors>();

            // Get rotation and translation from stella_vslam
            if (camera_pose) {
                // camera_pose points to Twc (camera in world coords)
                // We need Htw (torso to world), so we'll use the camera pose directly
                // Convert Mat44_t to Eigen::Isometry3d
                Eigen::Isometry3d Hwc = Eigen::Isometry3d::Identity();

                // Extract rotation and translation from the Mat44_t
                const auto& pose_matrix = *camera_pose;
                Hwc.translation() = Eigen::Vector3d(pose_matrix(0, 3), pose_matrix(1, 3), pose_matrix(2, 3));
                Hwc.linear() = pose_matrix.block<3, 3>(0, 0);

                // Obtain rpy from the camera pose
                Eigen::Vector3d rpy = mat_to_rpy_intrinsic(Hwc.rotation());

                // Rotate the camera pose to the torso frame (x forward, y left, z up), reconstructing the matrix
                Eigen::Matrix3d R = rpy_intrinsic_to_mat(Eigen::Vector3d(rpy.z(), -rpy.x(), -rpy.y()));

                // positive y is negative is negative x
                // negatve x is negative yaw
                // positive z is negative pitch y
                Hwc.linear() = R;

                // For now, use Hwc as Htw (assuming camera is mounted on torso)
                // You might need to adjust this based on your robot's camera mounting
                sensors->Htw = Hwc.inverse();
            }

            emit(std::move(sensors));
        });
    }

}  // namespace module::input
