#ifndef MODULE_VISION_YOLO_HPP
#define MODULE_VISION_YOLO_HPP

#include <Eigen/Core>
#include <nuclear>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

namespace module::vision {

    class Yolo : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief NMS threshold for filtering out overlapping bounding boxes
            double nms_threshold = 0.5;
            /// @brief NMS confidence (score) threshold for filtering out low confidence detections
            double nms_score_threshold = 0.5;
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
            Eigen::Vector4d colour = Eigen::Vector4d(1, 1, 1, 1);
            /// @brief Confidence threshold
            double confidence_threshold = 0.0;
        };

        /// @brief The objects that the Yolo model can detect
        std::vector<Object> objects = {{"ball", Eigen::Vector4d(1, 1, 1, 1), 0.0},
                                       {"goal post", Eigen::Vector4d(1, 0, 1, 1), 0.0},
                                       {"robot", Eigen::Vector4d(1, 0.5, 0, 1), 0.0},
                                       {"L-intersection", Eigen::Vector4d(1, 0, 0, 1), 0.0},
                                       {"T-intersection", Eigen::Vector4d(0, 1, 0, 1), 0.0},
                                       {"X-intersection", Eigen::Vector4d(0, 0, 1, 1), 0.0}};


    public:
        /// @brief Called by the powerplant to build and setup the Yolo reactor.
        explicit Yolo(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::vision

#endif  // MODULE_VISION_YOLO_HPP
