#include "DisplayTest.h"
#include "messages/PlainMessage.h"
#include "utility/NUbugger/NUgraph.h"

using utility::NUbugger::graph;

namespace modules {
    namespace support {
        DisplayTest::DisplayTest(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Trigger<Every<100, std::chrono::milliseconds>>>([this](const time_t&) {

                /*std::string value = "testing";
                NUClear::log<NUClear::DEBUG>(value);

                auto message = std::make_unique<messages::PlainMessage>();
                message->value = value;
                emit(std::move(message));*/

                /*emit(std::make_unique<DataPoint>(DataPoint{
                    "Debug", {(float) rand() / RAND_MAX * 100}
                }));*/
                //generate("Debug", {1});
                float value = float(rand()) / RAND_MAX * 100;
                emit(graph("Debug", value));

            });
        }
    }
}
