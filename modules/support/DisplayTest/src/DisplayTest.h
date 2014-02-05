#ifndef MODULES_DISPLAY_TEST_H
#define MODULES_DISPLAY_TEST_H

#include <nuclear>

namespace modules {
    namespace support {

        class DisplayTest : public NUClear::Reactor {
        public:
            explicit DisplayTest(std::unique_ptr<NUClear::Environment> environment);
        };

    }
}
#endif

