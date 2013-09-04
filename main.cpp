#include <NUClear.h>
#include <signal.h>
#include <math.h>

#include "ConfigSystem.h"
#include "DarwinPlatform.h"
#include "DarwinCameraReader.h"
#include "DarwinMotionManager.h"
#include "ScriptEngine.h"
#include "eSpeak.h"
#include "NUBugger.h"
#include "PartyDarwin.h"
#include "ScriptTuner.h"
#include "AudioInput.h"

struct SegmentationFault : public std::exception {};

// This namespace holds our signal handling code
namespace {
    NUClear::PowerPlant* powerplant;
    bool run = false;

    void signalHandler(int signal) {

        std::cout << std::endl << "Shutdown Command Sent" << std::endl;

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
}

int main(int argc, char *argv[]) {

    NUClear::PowerPlant::Configuration config;

    config.threadCount = 4;

    NUClear::PowerPlant plant(config);
    powerplant = &plant;

    // If we get interrupted (ctrl c) then tell the system to shutdown gracefully, on the second time just kill it
    signal(SIGINT, signalHandler);

    // For segfaults and abort signals, convert them to exceptions and throw them, they will then be caught and not cause errors
    signal(SIGSEGV, segfaultConverter);

    plant.install<modules::ConfigSystem>();
    //plant.install<modules::DarwinPlatform>();
    //plant.install<modules::DarwinCameraReader>();
    plant.install<modules::DarwinMotionManager>();
    plant.install<modules::ScriptEngine>();
    plant.install<modules::eSpeak>();
    //plant.install<modules::AudioInput>();
    plant.install<modules::NUBugger>();
    plant.install<modules::ScriptTuner>();
    plant.install<modules::PartyDarwin>();

    plant.start();
}

