/*! @file Module.h
    @brief Test configurable module class.
    
    @author Mitchell Metcalfe
*/

#ifndef Module_H
#define Module_H

#include "Configurable.h"
#include "ConfigManager.h"
extern ConfigSystem::ConfigManager* config;

class Module : Configurable
{
private:
    // some example parameters:
    long        longParam1;
    double      doubleParam1;
    std::string stringParam1;

    void loadConfig();
    void updateConfig();
    // void updateConfig(
    //     const std::string& paramPath,
    //     const std::string& paramName
    //     );
};

#endif

