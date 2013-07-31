#include "DarwinCamera.h"
namespace modules {
    
    DarwinCamera::DarwinCamera(NUClear::PowerPlant& plant) : Reactor(plant) {
        on<Trigger<Every<20, std::chrono::milliseconds>>>([](const time_t& time) {
            
        });
    }
}