#ifndef MODULES_DARWINMOTORS_H
#define MODULES_DARWINMOTORS_H

#include <NUClear.h>

namespace modules {

    class DarwinMotors : public NUClear::Reactor {
    public:
        DarwinMotors(NUClear::PowerPlant& plant);
    };
}
#endif

