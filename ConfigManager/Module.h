/*! @file Module.h
    @brief Test configurable module class.
    
    @author Mitchell Metcalfe
*/

#ifndef Module_H
#define Module_H

#include "Configurable.h"

class Module : Configurable
{
private:
    // some example parameters:
    int         intParam1;
    float       floatParam1;
    std::string stringParam1;

    void loadConfig();
    void updateConfig(
        const std::string& paramPath,
        const std::string& paramName
        );
};

#endif

