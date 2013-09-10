#include <NUClear.h>

#include "ConfigSystem.h"
#include "DarwinPlatform.h"
#include "DarwinMotionManager.h"
#include "ScriptEngine.h"
#include "ScriptRunner.h"
#include "NUBugger.h"
#include "PartyDarwin.h"

int main(int argc, char *argv[]) {

    NUClear::PowerPlant::Configuration config;

    config.threadCount = 4;

    NUClear::PowerPlant plant(config, argc, const_cast<const char**>(argv));

    plant.install<modules::ConfigSystem>();
    plant.install<modules::DarwinPlatform>();
    plant.install<modules::DarwinMotionManager>();
    plant.install<modules::ScriptEngine>();
    plant.install<modules::ScriptRunner>();
    plant.install<modules::NUBugger>();
    plant.install<modules::PartyDarwin>();

    plant.start();
}

