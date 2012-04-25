#ifndef DEBUG_H
#define DEBUG_H

#include <iostream>
#include <fstream>

#include "debugvisionverbosities.h"

#define DATA_DIR (std::string(getenv("HOME")) + std::string("/nubot/"))
#define CONFIG_DIR (DATA_DIR + std::string("Config/Darwin/"))
#define RULE_DIR (DATA_DIR + std::string("Rules/"))

//extern std::ofstream debug;
//extern std::ofstream errorlog;

#define debug std::cout
#define errorlog std::cerr

#endif // DEBUG_H
