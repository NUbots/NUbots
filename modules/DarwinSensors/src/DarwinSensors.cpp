#include "DarwinSensors.h"
namespace modules {
    
    DarwinSensors::DarwinSensors(NUClear::PowerPlant& plant) : Reactor(plant) {
        on<Trigger<Every<20, std::chrono::milliseconds>>>([](const time_t& time) {
            
        });
    }
}