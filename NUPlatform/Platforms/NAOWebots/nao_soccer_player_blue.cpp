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

#include <sstream>
#include <string.h>
#include <stdlib.h>

using namespace std;
ofstream debug;
ofstream errorlog;

int getPlayerNumber(int argc, const char *argv[])
{
    int port = -1;
    for (int i = 1; i < argc; i++) 
    {
        if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            port = atoi(argv[i + 1]);
            break;
        }
    }
    
    if (port == -1) 
        cout << "Error: could not find port number in controllerArgs" << endl;
    return (port % 10) + 1;
}

int main(int argc, const char *argv[]) 
{
    stringstream filename;
    filename << getPlayerNumber(argc, argv) << "debug.log";
    debug.open(filename.str().c_str());    // I need to use a different name for each robot!
    stringstream errorlogname;
    errorlogname << getPlayerNumber(argc, argv) << "debug.log";
    errorlog.open(errorlogname.str().c_str());
    NUbot* nubot = new NUbot(argc, argv);
    nubot->run();
    delete nubot;
}
