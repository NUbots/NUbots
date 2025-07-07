#ifndef MODULE_VISION_FISHEYEPROJECTION_HPP
#define MODULE_VISION_FISHEYEPROJECTION_HPP

#include <Eigen/Core>
#include <nuclear>

namespace module::vision {

    class FisheyeProjection : public NUClear::Reactor {
    public:
        explicit FisheyeProjection(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct Config {
            NUClear::LogLevel log_level = NUClear::LogLevel::INFO;
        } cfg;
    };

}  // namespace module::vision

#endif  // MODULE_VISION_FISHEYEPROJECTION_HPP
