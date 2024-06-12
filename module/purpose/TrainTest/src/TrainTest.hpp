#ifndef MODULE_PURPOSE_TRAINTEST_HPP
#define MODULE_PURPOSE_TRAINTEST_HPP

#include <nuclear>


namespace module::purpose {

    class TrainTest : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the TrainTest reactor.
        explicit TrainTest(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_TRAINTEST_HPP
