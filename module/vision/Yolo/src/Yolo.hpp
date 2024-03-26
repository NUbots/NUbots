#ifndef MODULE_VISION_YOLO_HPP
#define MODULE_VISION_YOLO_HPP

#include <nuclear>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

namespace module::vision {

    class Yolo : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief The confidence threshold for the ball
            double ball_confidence_threshold = 0.0;
            /// @brief The confidence threshold for the goal
            double goal_confidence_threshold = 0.0;
            /// @brief The confidence threshold for the robot
            double robot_confidence_threshold = 0.0;
            /// @brief The confidence threshold for the field intersections (L, T, X)
            double intersection_confidence_threshold = 0.0;
        } cfg;

        /// @brief OpenVINO compiled model, used to create inference request object
        ov::CompiledModel compiled_model{};

        /// @brief Inference request, used to run the model (inference)
        ov::InferRequest infer_request{};

        /// @brief Object struct for storing name and colour
        struct Object {
            /// @brief Class name
            std::string name = "";
            /// @brief Colour for bounding box
            cv::Scalar colour = cv::Scalar(255, 255, 255);
        };

        /// @brief The objects that the Yolo model can detect
        const std::vector<Object> objects = {{"ball", cv::Scalar(255, 255, 255)},
                                             {"goal post", cv::Scalar(255, 0, 255)},
                                             {"robot", cv::Scalar(255, 127.5, 0)},
                                             {"L-intersection", cv::Scalar(255, 0, 0)},
                                             {"T-intersection", cv::Scalar(0, 255, 0)},
                                             {"X-intersection", cv::Scalar(0, 0, 255)}};

    public:
        /// @brief Called by the powerplant to build and setup the Yolo reactor.
        explicit Yolo(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::vision

#endif  // MODULE_VISION_YOLO_HPP
