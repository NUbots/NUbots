#include "ORBSLAM.hpp"

#include "clock/clock.hpp"

#include "extension/Configuration.hpp"

#include "message/input/Image.hpp"

#include "utility/vision/Vision.hpp"
#include "utility/vision/fourcc.hpp"

namespace module::input {

    using extension::Configuration;

    using message::input::Image;

    ORBSLAM::ORBSLAM(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("ORBSLAM.yaml").then([this](const Configuration& config) {
            // Use configuration here from file ORBSLAM.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // Initialise SLAM system - Pangolin viewer is currently not applicable in NUBots docker.
            auto sys = ORB_SLAM3::System(config["vocabulary"].as<std::string>(),
                                         config["settings"].as<std::string>(),
                                         ORB_SLAM3::System::eSensor::MONOCULAR,
                                         false);
        });


        // on<Trigger<Image>, Single>().then([this](const Image& image) {
        //     // Image frame tracker.
        //     static uint frame = 0;
        //     log<NUClear::DEBUG>(fmt::format("--- Image received : {} --- ", frame++));

        //     // Constant image data from protobuf image.
        //     const int imageID                            = image.id;
        //     const std::string imageName                  = image.name;
        //     const int imageDimensionsX                   = image.dimensions.x();
        //     const int imageDimensionsY                   = image.dimensions.y();
        //     const int imageFourColourCode                = image.format;
        //     const std::string imageFourColourName        = utility::vision::fourcc(imageFourColourCode);
        //     const std::chrono::time_point imageTimePoint = image.timestamp;

        //     // Cast protobuff input image to greyscale output image.
        //     cv::Mat cvImageInput;
        //     cv::Mat cvImageOutput;
        //     switch (imageFourColourCode) {
        //         case utility::vision::FOURCC::RGB3:
        //         case utility::vision::FOURCC::JPEG:
        //             cvImageInput =
        //                 cv::Mat(imageDimensionsY, imageDimensionsX, CV_8UC3,
        //                 const_cast<uint8_t*>(image.data.data()));
        //             cv::cvtColor(cvImageInput, cvImageOutput, cv::COLOR_RGB2GRAY);
        //             break;
        //         case utility::vision::fourcc("BGR3"):  // BGR3 not available in utility::vision::FOURCC.
        //             cvImageInput =
        //                 cv::Mat(imageDimensionsY, imageDimensionsX, CV_8UC3,
        //                 const_cast<uint8_t*>(image.data.data()));
        //             cv::cvtColor(cvImageInput, cvImageOutput, cv::COLOR_BGR2GRAY);
        //             break;
        //         case utility::vision::FOURCC::GREY:
        //             cvImageInput =
        //                 cv::Mat(imageDimensionsY, imageDimensionsX, CV_8UC1,
        //                 const_cast<uint8_t*>(image.data.data()));
        //             cvImageOutput = cvImageInput.clone();
        //             break;
        //         default:
        //             log<NUClear::WARN>(fmt::format("Image format not supported: {}", imageFourColourName));
        //             mutex.unlock();
        //             return;
        //     }


        //     // Update the slam system tracking with the new black and white image.
        //     Sophus::SE3f Hcw_sophus =
        //         slamSystem->TrackMonocular(cvImageOutput, imageTimePoint.time_since_epoch().count());


        //     // Convert Sophus::SE3f to Eigen::Isometry3d.
        //     // Eigen::Isometry3d Hcw = Eigen::Isometry3d(Hcw_sophus.matrix().cast<double>());
        //     // log<NUClear::DEBUG>(fmt::format("Hcw: {}", Hcw.matrix()));
        //     // Attach Eigen matrix to message.
        //     // message->Hcw = eigenMatrix;

        //     // Emit message globally.
        //     // emit(std::move(message));
        //     // Emit message locally.
        //     // emit<Scope::DIRECT>(std::move(message));
        // });
    }

}  // namespace module::input
