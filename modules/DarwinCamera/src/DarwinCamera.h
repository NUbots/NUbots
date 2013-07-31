#ifndef MODULES_DARWINCAMERA_H
#define MODULES_DARWINCAMERA_H

#include <NUClear.h>

namespace modules {

    class DarwinCamera : public NUClear::Reactor {
    public:
        DarwinCamera(NUClear::PowerPlant& plant);
    };
}
#endif

