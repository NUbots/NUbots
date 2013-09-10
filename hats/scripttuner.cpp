#include <NUClear.h>

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

int main(int argc, char *argv[]) {

    NUClear::PowerPlant::Configuration config;

    config.threadCount = 4;

    NUClear::PowerPlant plant(config, argc, const_cast<const char**>(argv));

    plant.install<modules::DarwinPlatform>();
    plant.install<modules::DarwinMotionManager>();
    plant.install<modules::NUBugger>();
    plant.install<modules::ScriptTuner>();
    plant.install<modules::PartyDarwin>();

    plant.start();
}

