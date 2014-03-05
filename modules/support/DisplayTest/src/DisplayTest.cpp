#include "DisplayTest.h"
#include "utility/NUbugger/NUgraph.h"

using utility::NUbugger::graph;

namespace modules {
    namespace support {
        DisplayTest::DisplayTest(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            // TODO: remove - just for debugging the graph
            /*on<Trigger<Every<100, std::chrono::milliseconds>>>([this](const time_t&) {

                float value = float(rand()) / RAND_MAX * 100;
                emit(graph("Debug", value));

            });*/
        }
    }
}
