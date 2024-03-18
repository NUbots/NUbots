#ifndef MODULE_VISION_YOLO_HPP
#define MODULE_VISION_YOLO_HPP

#include <nuclear>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

namespace module::vision {

    std::vector<cv::Scalar> colors = {cv::Scalar(0, 0, 255),
                                      cv::Scalar(0, 255, 0),
                                      cv::Scalar(255, 0, 0),
                                      cv::Scalar(255, 100, 50),
                                      cv::Scalar(50, 100, 255),
                                      cv::Scalar(255, 50, 100)};

    const std::vector<std::string> class_names =
        {"ball", "goal post", "robot", "L-intersection", "T-intersection", "X-intersection"};


    // Keep the ratio before resize
    cv::Mat letterbox(const cv::Mat& source) {
        int col        = source.cols;
        int row        = source.rows;
        int _max       = MAX(col, row);
        cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
        source.copyTo(result(cv::Rect(0, 0, col, row)));
        return result;
    }

    class Yolo : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            std::string model_path                   = "";
            double ball_confidence_threshold         = 0.0;
            double goal_confidence_threshold         = 0.0;
            double robot_confidence_threshold        = 0.0;
            double intersection_confidence_threshold = 0.0;

            std::string image_format = "";
            std::string device       = "";
        } cfg;

        /// @brief OpenVINO Core
        ov::Core core;

        /// @brief Compiled model
        ov::CompiledModel compiled_model;

        /// @brief Inference request
        ov::InferRequest infer_request;

    public:
        /// @brief Called by the powerplant to build and setup the Yolo reactor.
        explicit Yolo(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::vision

#endif  // MODULE_VISION_YOLO_HPP
