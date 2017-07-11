#include "ServoTest.h"

#include "extension/Configuration.h"

#include "utility/input/ServoID.h"

namespace module {
namespace behaviour {
    namespace tools {

        using extension::Configuration;

        using ServoID = utility::input::ServoID;


        ServoTest::ServoTest(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Configuration>("ServoTest.yaml").then([this] /*(const Configuration& config)*/ {
                // Use configuration here from file ServoTest.yaml
            });
        }
    }  // namespace tools
}  // namespace behaviour
}  // namespace module
