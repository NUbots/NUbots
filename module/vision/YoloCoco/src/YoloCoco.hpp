#ifndef MODULE_VISION_YOLOCOCO_HPP
#define MODULE_VISION_YOLOCOCO_HPP

#include <Eigen/Core>
#include <nuclear>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

namespace module::vision {

    class YoloCoco : public NUClear::Reactor {
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
        std::vector<Object> objects = {{"person", Eigen::Vector4d(1, 1, 1, 1), 0.0},
                                       {"bicycle", Eigen::Vector4d(1, 0, 1, 1), 0.0},
                                       {"car", Eigen::Vector4d(1, 0.5, 0, 1), 0.0},
                                       {"motorcycle", Eigen::Vector4d(1, 0, 0, 1), 0.0},
                                       {"airplane", Eigen::Vector4d(0, 1, 0, 1), 0.0},
                                       {"bus", Eigen::Vector4d(0, 0, 1, 1), 0.0},
                                       {"train", Eigen::Vector4d(1, 1, 1, 1), 0.0},
                                       {"truck", Eigen::Vector4d(1, 0, 1, 1), 0.0},
                                       {"boat", Eigen::Vector4d(1, 0.5, 0, 1), 0.0},
                                       {"traffic light", Eigen::Vector4d(1, 0, 0, 1), 0.0},
                                       {"fire hydrant", Eigen::Vector4d(0, 1, 0, 1), 0.0},
                                       {"stop sign", Eigen::Vector4d(0, 0, 1, 1), 0.0},
                                       {"parking meter", Eigen::Vector4d(1, 1, 1, 1), 0.0},
                                       {"bench", Eigen::Vector4d(1, 0, 1, 1), 0.0},
                                       {"bird", Eigen::Vector4d(1, 0.5, 0, 1), 0.0},
                                       {"cat", Eigen::Vector4d(1, 0, 0, 1), 0.0},
                                       {"dog", Eigen::Vector4d(0, 1, 0, 1), 0.0},
                                       {"horse", Eigen::Vector4d(0, 0, 1, 1), 0.0},
                                       {"sheep", Eigen::Vector4d(1, 1, 1, 1), 0.0},
                                       {"cow", Eigen::Vector4d(1, 0, 1, 1), 0.0},
                                       {"elephant", Eigen::Vector4d(1, 0.5, 0, 1), 0.0},
                                       {"bear", Eigen::Vector4d(1, 0, 0, 1), 0.0},
                                       {"zebra", Eigen::Vector4d(0, 1, 0, 1), 0.0},
                                       {"giraffe", Eigen::Vector4d(0, 0, 1, 1), 0.0},
                                       {"backpack", Eigen::Vector4d(1, 1, 1, 1), 0.0},
                                       {"umbrella", Eigen::Vector4d(1, 0, 1, 1), 0.0},
                                       {"handbag", Eigen::Vector4d(1, 0.5, 0, 1), 0.0},
                                       {"tie", Eigen::Vector4d(1, 0, 0, 1), 0.0},
                                       {"suitcase", Eigen::Vector4d(0, 1, 0, 1), 0.0},
                                       {"frisbee", Eigen::Vector4d(0, 0, 1, 1), 0.0},
                                       {"skis", Eigen::Vector4d(1, 1, 1, 1), 0.0},
                                       {"snowboard", Eigen::Vector4d(1, 0, 1, 1), 0.0},
                                       {"sports ball", Eigen::Vector4d(1, 0.5, 0, 1), 0.0},
                                       {"kite", Eigen::Vector4d(1, 0, 0, 1), 0.0},
                                       {"baseball bat", Eigen::Vector4d(0, 1, 0, 1), 0.0},
                                       {"baseball glove", Eigen::Vector4d(0, 0, 1, 1), 0.0},
                                       {"skateboard", Eigen::Vector4d(1, 1, 1, 1), 0.0},
                                       {"surfboard", Eigen::Vector4d(1, 0, 1, 1), 0.0},
                                       {"tennis racket", Eigen::Vector4d(1, 0.5, 0, 1), 0.0},
                                       {"bottle", Eigen::Vector4d(1, 0, 0, 1), 0.0},
                                       {"wine glass", Eigen::Vector4d(0, 1, 0, 1), 0.0},
                                       {"cup", Eigen::Vector4d(0, 0, 1, 1), 0.0},
                                       {"fork", Eigen::Vector4d(1, 1, 1, 1), 0.0},
                                       {"knife", Eigen::Vector4d(1, 0, 1, 1), 0.0},
                                       {"spoon", Eigen::Vector4d(1, 0.5, 0, 1), 0.0},
                                       {"bowl", Eigen::Vector4d(1, 0, 0, 1), 0.0},
                                       {"banana", Eigen::Vector4d(0, 1, 0, 1), 0.0},
                                       {"apple", Eigen::Vector4d(0, 0, 1, 1), 0.0},
                                       {"sandwich", Eigen::Vector4d(1, 1, 1, 1), 0.0},
                                       {"orange", Eigen::Vector4d(1, 0, 1, 1), 0.0},
                                       {"broccoli", Eigen::Vector4d(1, 0.5, 0, 1), 0.0},
                                       {"carrot", Eigen::Vector4d(1, 0, 0, 1), 0.0},
                                       {"hot dog", Eigen::Vector4d(0, 1, 0, 1), 0.0},
                                       {"pizza", Eigen::Vector4d(0, 0, 1, 1), 0.0},
                                       {"donut", Eigen::Vector4d(1, 1, 1, 1), 0.0},
                                       {"cake", Eigen::Vector4d(1, 0, 1, 1), 0.0},
                                       {"chair", Eigen::Vector4d(1, 0.5, 0, 1), 0.0},
                                       {"couch", Eigen::Vector4d(1, 0, 0, 1), 0.0},
                                       {"potted plant", Eigen::Vector4d(0, 1, 0, 1), 0.0},
                                       {"bed", Eigen::Vector4d(0, 0, 1, 1), 0.0},
                                       {"dining table", Eigen::Vector4d(1, 1, 1, 1), 0.0},
                                       {"toilet", Eigen::Vector4d(1, 0, 1, 1), 0.0},
                                       {"tv", Eigen::Vector4d(1, 0.5, 0, 1), 0.0},
                                       {"laptop", Eigen::Vector4d(1, 0, 0, 1), 0.0},
                                       {"mouse", Eigen::Vector4d(0, 1, 0, 1), 0.0},
                                       {"remote", Eigen::Vector4d(0, 0, 1, 1), 0.0},
                                       {"keyboard", Eigen::Vector4d(1, 1, 1, 1), 0.0},
                                       {"cell phone", Eigen::Vector4d(1, 0, 1, 1), 0.0},
                                       {"microwave", Eigen::Vector4d(1, 0.5, 0, 1), 0.0},
                                       {"oven", Eigen::Vector4d(1, 0, 0, 1), 0.0},
                                       {"toaster", Eigen::Vector4d(0, 1, 0, 1), 0.0},
                                       {"sink", Eigen::Vector4d(0, 0, 1, 1), 0.0},
                                       {"refrigerator", Eigen::Vector4d(1, 1, 1, 1), 0.0},
                                       {"book", Eigen::Vector4d(1, 0, 1, 1), 0.0},
                                       {"clock", Eigen::Vector4d(1, 0.5, 0, 1), 0},
                                       {"vase", Eigen::Vector4d(1, 0, 0, 1), 0.0},
                                       {"scissors", Eigen::Vector4d(0, 1, 0, 1), 0.0},
                                       {"teddy bear", Eigen::Vector4d(0, 0, 1, 1), 0.0},
                                       {"hair drier", Eigen::Vector4d(1, 1, 1, 1), 0.0},
                                       {"toothbrush", Eigen::Vector4d(1, 0, 1, 1), 0.0}};

    public:
        /// @brief Called by the powerplant to build and setup the Yolo reactor.
        explicit Yolo(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::vision

#endif  // MODULE_VISION_YOLOCOCO_HPP
