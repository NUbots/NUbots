#include "ServoTest.h"

#include "message/support/Configuration.h"
#include "message/input/ServoID.h"



namespace module {
namespace behaviour {
namespace tools {

    using message::support::Configuration;
    using message::input::ServoID;


    ServoTest::ServoTest(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("ServoTest.yaml").then([this] /*(const Configuration& config)*/ {
            // Use configuration here from file ServoTest.yaml
        });
    }
}
}
}
