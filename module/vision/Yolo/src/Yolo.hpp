#ifndef MODULE_VISION_YOLO_HPP
#define MODULE_VISION_YOLO_HPP

#include <nuclear>
#include <openvino/openvino.hpp>

namespace module::vision {

    class Yolo : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            std::string model_path = "";
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
