#ifndef MODULE_VISION_YOLO_HPP
#define MODULE_VISION_YOLO_HPP

#include <nuclear>

#include "inference.h"

namespace module::vision {

    class Yolo : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            std::string model_path = "";
        } cfg;

        Inference inf;

    public:
        /// @brief Called by the powerplant to build and setup the Yolo reactor.
        explicit Yolo(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::vision

#endif  // MODULE_VISION_YOLO_HPP
