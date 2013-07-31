#include <NUClear.h>
#include "DarwinMotors.h"
#include "DarwinSensors.h"
#include "DarwinCamera.h"

int main(int argc, char *argv[]) {
    
    NUClear::PowerPlant plant;
    
    plant.install<modules::DarwinMotors>();
    plant.install<modules::DarwinSensors>();
    plant.install<modules::DarwinCamera>();
    
    plant.start();
}