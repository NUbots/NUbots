//---------------------------------------------------------------------------------------
//  File:         nao_soccer_player_blue.cpp (to be used in a Webots controllers)
//  Description:  This is a full C++ controller example for Robotstadium
//                It selects and runs the correct player type: FieldPlayer or GoalKeeper
//  Project:      Robotstadium, the online robot soccer competition
//  Author:       Yvan Bourquin - www.cyberbotics.com
//  Date:         May 11, 2009
//  Changes:      
//---------------------------------------------------------------------------------------

#include "NUbot.h"
#include "debug.h"
#include "nubotdataconfig.h"

#include <sstream>
#include <string.h>
#include <stdlib.h>
#include <iomanip>
#include <ctime>

using namespace std;
ofstream debug;
ofstream errorlog;

int getLogNumber(int argc, const char *argv[])
{
    if (argc < 3)
        return 0;
    
    return atoi(argv[1]) + 1 + 10*atoi(argv[2]);
}

int main(int argc, const char *argv[]) 
{
    stringstream filename_prefix, filename_postfix;
    time_t timenow;
    time(&timenow);
    struct tm* timeinfo = localtime(&timenow);
    char buffer[80];
    strftime(buffer, 80, "%Y%m%d %H%M%S ", timeinfo);
    
    filename_prefix << buffer << " " << setfill('0') << setw(2) << getLogNumber(argc, argv);
    filename_postfix << ".log";
    
    debug.open((DATA_DIR + filename_prefix.str() + "debug" + filename_postfix.str()).c_str());
    errorlog.open((DATA_DIR + filename_prefix.str() + "error" + filename_postfix.str()).c_str());
    
    NUbot* nubot = new NUbot(argc, argv);
    nubot->run();
    delete nubot;
}
