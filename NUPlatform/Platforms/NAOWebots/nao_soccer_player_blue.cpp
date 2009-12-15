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

using namespace std;

ofstream debug;

int main(int argc, const char *argv[]) 
{
    debug.open("debug.log");
    NUbot* nubot = new NUbot(argc, argv);
    nubot->run();
    delete nubot;
}
