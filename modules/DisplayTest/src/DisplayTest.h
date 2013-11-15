#ifndef MODULES_DISPLAY_TEST_H
#define MODULES_DISPLAY_TEST_H

#include <NUClear.h>

namespace modules {

    class DisplayTest : public NUClear::Reactor {
    public:
        explicit DisplayTest(NUClear::PowerPlant* plant);
    };
}
#endif

