#include <NUClear.h>
#include <signal.h>

#include "DarwinPlatform.h"
#include "DarwinCamera.h"
#include "NUBugger.h"

struct SegmentationFault : public std::exception {};
struct AbortSignal : public std::exception {};

// This namespace holds our signal handling code
namespace {
    NUClear::PowerPlant* powerplant;
    bool run = false;
    
    void signalHandler(int signal) {

        std::cout << "Shutdown Command Sent" << std::endl;

        // On our first interrupt, tell the system to shutdown
        if(!run) {
            powerplant->shutdown();
            run = true;
        }
        // If they do it again, murder is the answer
        else {
            exit(1);
        }
    }
    
    void segfaultConverter(int signal) {
        std::cout << "Segmentation Fault" << std::endl;
        throw SegmentationFault();
    }
    
    void abortfaultConverter(int signal) {
        throw AbortSignal();
    }
}

int main(int argc, char *argv[]) {
    
    NUClear::PowerPlant plant;
    powerplant = &plant;
    
    // If we get interrupted (ctrl c) then tell the system to shutdown gracefully, on the second time just kill it
    signal(SIGINT, signalHandler);
    
    // For segfaults and abort signals, convert them to exceptions and throw them, they will then be caught and not cause errors
    signal(SIGSEGV, segfaultConverter);

    plant.install<modules::DarwinPlatform>();
    plant.install<modules::DarwinCamera>();
    plant.install<modules::NUBugger>();
    
    plant.start();
}

