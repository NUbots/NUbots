#ifndef MODULE_PURPOSE_TRAINTEST_HPP
#define MODULE_PURPOSE_TRAINTEST_HPP

#include <nuclear>
#include <random>

#include "utility/rl/TestEnvironment.h"

namespace module::purpose {

    // Vector of tensors.
    using VT = std::vector<torch::Tensor>;


    class TrainTest : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
        } cfg;

        // Random engine.
        std::random_device rd;
        std::mt19937 re                      = std::mt19937(rd());
        std::uniform_int_distribution<> dist = std::uniform_int_distribution<>(-5, 5);

        // Environment.
        double x = double(dist(re));  // goal x pos
        double y = double(dist(re));  // goal y pos
        TestEnvironment env{x, y};

        // Training loop.
        uint n_iter          = 10000;
        uint n_steps         = 2048;
        uint n_epochs        = 15;
        uint mini_batch_size = 512;
        uint ppo_epochs      = 4;
        double beta          = 1e-3;

        VT states;
        VT actions;
        VT rewards;
        VT dones;

        VT log_probs;
        VT returns;
        VT values;

    public:
        /// @brief Called by the powerplant to build and setup the TrainTest reactor.
        explicit TrainTest(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_TRAINTEST_HPP
