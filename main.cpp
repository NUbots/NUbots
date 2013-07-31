#include <NUClear.h>
#include "DarwinMotors.h"

int main(int argc, char *argv[]) {
    
    NUClear::PowerPlant plant;
    
    plant.install<modules::DarwinMotors>();
    
    plant.start();
}