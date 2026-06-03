#ifndef MODULE_LOCALISATION_NURAL_HPP
#define MODULE_LOCALISATION_NURAL_HPP

#include <nuclear>
#include <openvino/openvino.hpp>

namespace module::localisation {

    class NUral : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Path to the NUral ONNX model file
            std::string model_path;
        } cfg;

        ov::Core core;
        ov::CompiledModel compiled_model;
        ov::InferRequest infer_request;

    public:
        /// @brief Called by the powerplant to build and setup the NUral reactor.
        explicit NUral(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_NURAL_HPP
