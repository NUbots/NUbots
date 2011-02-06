#include "NUbot.h"

#include "../Robotis/Motors.h"
#include "NUbot/SenseMoveThread.h"

#include "debug.h"
#include "nubotdataconfig.h"

#include <iostream>
using namespace std;

ofstream debug;
ofstream errorlog;

int main(int argc, const char *argv[]) 
{
    debug.open((DATA_DIR + "debug.log").c_str());
    errorlog.open((DATA_DIR + "error.log").c_str());
                  
    NUbot* nubot = new NUbot(argc, argv);
    Motors::getInstance()->setSensorThread(nubot->m_sensemove_thread);
    nubot->run();
    delete nubot;
}
