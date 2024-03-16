#ifndef MODULE_INPUT_ORBSLAM_HPP
#define MODULE_INPUT_ORBSLAM_HPP

#include <nuclear>
#include <opencv2/core/core.hpp>

#include "../ORBSLAM3/include/ImuTypes.h"
#include "../ORBSLAM3/include/System.h"

namespace module::input {

    class ORBSLAM : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the ORBSLAM reactor.
        explicit ORBSLAM(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::input

#endif  // MODULE_INPUT_ORBSLAM_HPP
