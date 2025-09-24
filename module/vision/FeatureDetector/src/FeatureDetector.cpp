#include "FeatureDetector.hpp"
#include <opencv2/opencv.hpp>
#include <fmt/format.h>
#include "extension/Configuration.hpp"
#include "message/input/Image.hpp"
#include "message/input/Sensors.hpp"
#include "message/output/CompressedImage.hpp"
#include "utility/vision/Vision.hpp"
#include "utility/vision/fourcc.hpp"

namespace module::vision {

using extension::Configuration;
using message::input::Image;
using message::output::CompressedImage;

FeatureDetector::FeatureDetector(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Configuration>("FeatureDetector.yaml").then([this](const Configuration& config) {
        // Use configuration here from file FeatureDetector.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });

    on<Trigger<Image>, Single, MainThread>().then([this](const Image& image) {
        // Create output image from input image data
        int width  = image.dimensions.x();
        int height = image.dimensions.y();

        cv::Mat img_cv;
        cv::Mat img_rgb;
        switch (image.format) {
            case utility::vision::fourcc("BGR3"):
                img_cv = cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(image.data.data()));
                img_rgb = img_cv.clone();
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

        // Create output image for drawing features
        cv::Mat imgout = img_rgb.clone();
        double timestamp = NUClear::clock::now().time_since_epoch().count() / 1e9;
        std::vector<cv::KeyPoint> keypoints; // Store keypoints computed by FAST detector.
        cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(65, true, cv::FastFeatureDetector::TYPE_9_16);

        detector->detect(img_rgb, keypoints);

        int suppressionRadius = 20;
        std::vector<cv::KeyPoint> suppressed_keypoints;
        for (const auto& candidate : keypoints) {
            bool suppress = false;
            for (const auto& accepted : suppressed_keypoints) {
                double distance = cv::norm(candidate.pt - accepted.pt);
                if (distance < suppressionRadius) {
                    suppress = true;
                    break;
                }
            }
            if (!suppress) {
                suppressed_keypoints.push_back(candidate);
            }
        }
        keypoints = suppressed_keypoints;
        // Sort by response score (descending).
        std::sort(keypoints.begin(), keypoints.end(), [](const auto &a, const auto &b)
        {
            return a.response > b.response;
        });
        // Draw all detected corners (now suppressed) in green
        for (const auto &kp : keypoints)
        {
            cv::circle(imgout, kp.pt, 3, cv::Scalar(0, 255, 0), 1); // Green in BGR
        }
        // Limit to maxNumFeatures (default 100)
        const int maxNumFeatures = 20;
        if (keypoints.size() > static_cast<size_t>(maxNumFeatures))
        {
            keypoints.resize(maxNumFeatures);
        }

        // Print each feature
        for (size_t i = 0; i < keypoints.size(); ++i)
        {
            const auto& kp = keypoints[i];
            log<DEBUG>(fmt::format("{:<5} {:<10} {:<10} {:.8f}", i + 1, kp.pt.x, kp.pt.y, kp.response));
        }
        // Draw on output image
        for (size_t i = 0; i < keypoints.size(); ++i)
        {
            const auto& kp = keypoints[i];

            // Draw a red circle at the feature
            cv::circle(imgout, kp.pt, 4, cv::Scalar(0, 0, 255), 1);

            // Label the feature with its index number
            std::string label = std::to_string(i + 1);
            cv::putText(imgout, label,
                        kp.pt + cv::Point2f(5, -5), // offset text position
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.9,                   // font scale
                        cv::Scalar(255, 0, 0), // green text
                        2);                    // thickness
        }

        if (!imgout.empty()) {
            // Compress to JPEG using OpenCV
            std::vector<uint8_t> jpeg_data;
            std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 85};  // 85% quality
            cv::imencode(".jpg", imgout, jpeg_data, compression_params);

            // Create CompressedImage message
            auto compressed_img = std::make_unique<CompressedImage>();

            // Set basic fields
            compressed_img->format            = utility::vision::fourcc("JPEG");
            compressed_img->dimensions        = Eigen::Vector2<unsigned int>(imgout.cols, imgout.rows);
            compressed_img->id                = 1;
            compressed_img->name              = "FeatureDetector_frame";
            compressed_img->timestamp         = image.timestamp;  // Use the same timestamp as input image
            compressed_img->Hcw               = image.Hcw;        // Copy the camera transform
            compressed_img->lens.projection   = static_cast<int>(image.lens.projection);
            compressed_img->lens.focal_length = image.lens.focal_length;
            compressed_img->lens.fov          = image.lens.fov;
            compressed_img->lens.centre       = image.lens.centre;
            compressed_img->lens.k            = image.lens.k;

            // Copy compressed JPEG data
            compressed_img->data = jpeg_data;

            // Emit the compressed debug image
            emit(std::move(compressed_img));

            if (log_level <= DEBUG) {
                log<DEBUG>("Emitted FeatureDetector frame with FAST detections, size: ",
                           jpeg_data.size(),
                           " bytes");
            }
        }

    });
}

}  // namespace module::vision
