#ifndef MODULES_DARWINPLATFORM_H
#define MODULES_DARWINPLATFORM_H

#include <NUClear.h>

#include "Darwin.h"

#include "messages/DarwinMotors.h"
#include "messages/DarwinSensors.h"

namespace modules {
    
    class DarwinPlatform : public NUClear::Reactor {
    private:
        Darwin::Darwin m_darwin;
        
    public:
        DarwinPlatform(NUClear::PowerPlant& plant);
    };
}
#endif

