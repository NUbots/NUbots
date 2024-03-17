#include "ORBSLAM.hpp"

#include "clock/clock.hpp"

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"
#include "message/input/Sensors.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/math/euler.hpp"
#include "utility/vision/Vision.hpp"
#include "utility/vision/fourcc.hpp"


namespace module::input {

    using extension::Configuration;

    using utility::math::euler::EulerIntrinsicToMatrix;
    using utility::math::euler::MatrixToEulerIntrinsic;

    using message::input::Image;
    using message::input::Sensors;
    using message::platform::RawSensors;

    ORBSLAM::ORBSLAM(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("ORBSLAM.yaml").then([this](const Configuration& config) {
            // Use configuration here from file ORBSLAM.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // Initialise SLAM system
            slam_system = std::make_unique<ORB_SLAM3::System>(config["vocabulary"].as<std::string>(),
                                                              config["settings"].as<std::string>(),
                                                              ORB_SLAM3::System::eSensor::MONOCULAR,
                                                              false);
        });

        on<Trigger<Image>, With<RawSensors>, Single>().then([this](const Image& image, const RawSensors& raw_sensors) {
            // Convert input image to greyscale OpenCV image.
            cv::Mat img_in;
            cv::Mat img_out;
            switch (image.format) {
                case utility::vision::FOURCC::RGB3:
                case utility::vision::FOURCC::JPEG:
                    img_in = cv::Mat(image.dimensions.y(),
                                     image.dimensions.x(),
                                     CV_8UC3,
                                     const_cast<uint8_t*>(image.data.data()));
                    cv::cvtColor(img_in, img_out, cv::COLOR_RGB2GRAY);
                    break;
                case utility::vision::fourcc("BGR3"):  // BGR3 not available in utility::vision::FOURCC.
                    img_in = cv::Mat(image.dimensions.y(),
                                     image.dimensions.x(),
                                     CV_8UC3,
                                     const_cast<uint8_t*>(image.data.data()));
                    cv::cvtColor(img_in, img_out, cv::COLOR_BGR2GRAY);
                    break;
                case utility::vision::FOURCC::GREY:
                    img_in  = cv::Mat(image.dimensions.y(),
                                     image.dimensions.x(),
                                     CV_8UC1,
                                     const_cast<uint8_t*>(image.data.data()));
                    img_out = img_in.clone();
                    break;
                default:
                    log<NUClear::WARN>(
                        fmt::format("Image format not supported: {}", utility::vision::fourcc(image.format)));
                    powerplant.shutdown();
                    return;
            }

            // Update the slam system tracking with the new black and white image.
            std::vector<ORB_SLAM3::IMU::Point> imu_measurements;
            imu_measurements.push_back(ORB_SLAM3::IMU::Point(raw_sensors.accelerometer.x(),
                                                             raw_sensors.accelerometer.y(),
                                                             raw_sensors.accelerometer.z(),
                                                             raw_sensors.gyroscope.x(),
                                                             raw_sensors.gyroscope.y(),
                                                             raw_sensors.gyroscope.z(),
                                                             raw_sensors.timestamp.time_since_epoch().count()));
            Eigen::Isometry3d Hcw = Eigen::Isometry3d(
                slam_system->TrackMonocular(img_out, image.timestamp.time_since_epoch().count(), imu_measurements)
                    .matrix()
                    .cast<double>());
        });
    }

}  // namespace module::input
