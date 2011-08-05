#include "NUbot.h"

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
    nubot->run();
    delete nubot;
}
