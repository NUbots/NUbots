/*! 
    @file TestConfig.cpp
    @brief This is the cpp file for the testing of the configuration system for the NUbots.
  
    @author Sophie Calland, Mitchell Metcalfe
 
  Copyright (c) 2012 Sophie Calland, Mitchell Metcalfe
 
    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <exception>

#include "ConfigManager.h"
//#include "ConfigStorageManager.h"

using namespace ConfigSystem;


int main(void)
{
    // // Create path array for config system.
    // // Note: this should *definitely* be internal to the config system
    // // (perhaps only specify a base path + let the constructor handle 
    // // the filenames itself?).
    // std::string fNames[] = 
    //     {
    //         "ConfigurationFiles/behaviour.json"    ,
    //         "ConfigurationFiles/communication.json",
    //         "ConfigurationFiles/configuration.json",
    //         "ConfigurationFiles/localisation.json" ,
    //         "ConfigurationFiles/locomotion.json"   ,
    //         "ConfigurationFiles/vision.json"       ,
    //     };
    // //Read all config files. 
    ConfigManager config("defaultConfig");

    double sharpness;
    bool read = config.readDoubleValue("vision.Camera", "Sharpness", sharpness);
    
    //VECTOR TESTING:
    //1d long
    std::vector<long> dummy_vector;
    bool vread = config.readLongVectorValue1D("vision.Camera", "DummyVector", dummy_vector);
    
    
    // long awb;
    // read = config.readLongValue("vision.Camera", "AutoWhiteBalance", awb);

    // awb = 5;
    sharpness += 5;
    
    bool stored = config.storeDoubleValue("vision.Camera", "Sharpness", sharpness);
    
    std::cout << "readVectorLongValue = " << (vread? "success!" : "FAILED.") << std::endl;
    
    BOOST_FOREACH(const long &value, dummy_vector)
    {
    	std::cout << "value: " << value << std::endl;
    }
    
    BOOST_FOREACH(long &value, dummy_vector)
    {
    	value += 50;
    }
    
    std::cout << "storeDoubleValue = " << (stored? "success!" : "FAILED.") << std::endl;
    
    stored = config.storeLongVectorValue1D("vision.Camera", "DummyVector", dummy_vector);
    
    std::cout << "storeLongVectorValue = " << (stored? "success!" : "FAILED.") << std::endl;
    
    stored = config.storeLongValue("vision.Camera", "AutoWhiteBalance", 0);
    std::cout << "storeLongValue = " << (stored? "success!" : "FAILED.") << std::endl;
    
    config.saveConfiguration("newConfig");

    // // A property tree that holds ConfigParameter objects
    // boost::property_tree::basic_ptree<std::string, ConfigParameter, std::less<std::string> >
    //  cpTree;
    // ConfigParameter newCP(vt_double);
    // cpTree.put("root", newCP);

    return 0;
}
