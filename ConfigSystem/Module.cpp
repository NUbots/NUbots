/*! @file Module.cpp
    @brief 
    
    @author Mitchell Metcalfe
*/

#include "Module.h"

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
 boost::mt19937 seed(0);

void Module::loadConfig()
{
    double read_d;
    config->readDoubleValue("Testing.MM", "param_double", read_d);
    doubleParam1 = read_d;
}

void Module::updateConfig()
{
    loadConfig();
}
// void Module::updateConfig(
//     const std::string& paramPath,
//     const std::string& paramName
//     )
// {

//     loadConfig();
// }

