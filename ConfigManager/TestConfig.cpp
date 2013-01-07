/*! 
	@file TestConfig.cpp
    @brief This is the cpp file for the testing of the configuration system for the NUbots.
  
    @author Sophie Calland
 
  Copyright (c) 2012 Sophie Calland
 
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

//#include "ConfigStorageManager.h"
#include "ConfigEnforcer.h"
#include "ConfigManager.h"

using CONFIGURATION::ConfigStorageManager;
using CONFIGURATION::ConfigEnforcer;
using CONFIGURATION::ConfigManager;

int main(void)
{
	//ConfigStorageManager *config = new ConfigStorageManager();
	ConfigEnforcer *check = new ConfigEnforcer();
	
	std::string filename[ARR_SIZE];
	
	bool wsuccess = false, rsuccess = false, wasuccess = false, esuccess = false;
	bool editcheck1 = false, editcheck2 = false;
	
	bool intsuccess = false, intwritesuccess = false;
	
	int convertstuff = 0;
	
	
	filename[0] = "ConfigurationFiles/behaviour.json";
	filename[1] = "ConfigurationFiles/communication.json";
	filename[2] = "ConfigurationFiles/configuration.json";
	filename[3] = "ConfigurationFiles/localisation.json";
	filename[4] = "ConfigurationFiles/locomotion.json";
	filename[5] = "ConfigurationFiles/vision.json";
	
	//Read all config files. 
	ConfigManager *config = new ConfigManager(filename);
	
	intsuccess = config->readIntParam("root.behaviour.arbitrary_behave_constant", convertstuff);
	
	if(intsuccess) 	std::cout << "EPIC WINNING" << "\n";
	else			std::cout << "nope.avi" << "\n";
	
	std::cout << "string converted: " << convertstuff << "\n";
	
	std::cout << "changing to: " <<  50000 << "\n";
	
	//Can also change the type of variables. Have another class to safe proof this?
	intwritesuccess = config->storeIntParam("root.behaviour.arbitrary_behave_constant", 50000);
	
	config->printAll();
	
	if(intwritesuccess) std::cout << "SUCCESSFUL WRITE TO FILE" << "\n";
	else				std::cout  << "YOU LOSE" << "\n";
	
	
	
	
	
	
	
	
	
	//Testing just my stuff
	//Read all config files
	//rsuccess = config->fileReadAll(filename);
	
	
	/*filename[0] = "WRITE_ConfigurationFiles/behaviour_write.json";
	filename[1] = "WRITE_ConfigurationFiles/communication_write.json";
	filename[2] = "WRITE_ConfigurationFiles/configuration_write.json";
	filename[3] = "WRITE_ConfigurationFiles/localisation_write.json";
	filename[4] = "WRITE_ConfigurationFiles/locomotion_write.json";
	filename[5] = "WRITE_ConfigurationFiles/vision_write.json";*/
	
	//Editing a value
	//esuccess = config->editEntry("root.vision.arbitrary_vis_constant", "fffffaaa", "some awful type");

	//accessing a value and printing:
	/*CONFIGURATION::parameters<std::string> value_found;
	value_found = config->accessEntry("root.vision.arbitrary_vis_constant");
	std::cout << "VALUE: " << value_found.value << "\nTYPE: " << value_found.type << "\nUBOUND: " <<
				value_found.upper_bound << "\nLBOUND: " << value_found.lower_bound << "\n";
				
	for(int i = 0; i != value_found.possible_values.size(); i++)
	{
		std::cout << "VALUE" << i << ": " << value_found.possible_values[i] << "\n";
	}
	
	
	
	//Seeing if edited_values can be edited:
	CONFIGURATION::parameters<int> edited_values;
	edited_values.value = 50;
	edited_values.type = "int";
	edited_values.upper_bound = 50;
	edited_values.lower_bound = 50;
		
	edited_values.possible_values.push_back(500);
	edited_values.possible_values.push_back(50);
	edited_values.possible_values.push_back(2);
	
	editcheck1 = check->checkRangeValue(edited_values);
	editcheck2 = check->checkRangePossibleValues(edited_values);
	
	if(editcheck1) 	std::cout << "EDIT CHECK 1 TRUE" << std::endl;
	else			std::cout << "EDIT CHECK 1 FALSE" << std::endl;
	
	if(editcheck2) 	std::cout << "EDIT CHECK 2 TRUE" << std::endl;
	else			std::cout << "EDIT CHECK 2 FALSE" << std::endl;
	
	
							
	
	//Write all config files:
	wasuccess = config->fileWriteAll(false, filename);
	//wasuccess = config->fileWriteComponent(5, filename[5]);
	
	
	
	//WRITE ENTIRE TREE TO FILE:
	filename[0] = "testconfig_write.json";
	wsuccess = config->fileWriteAll(true, filename);
	
	if(rsuccess && wsuccess)
	{
		std::cout << "\nwinninnnng \n";
	}
	else
	{
		std::cout << "\nyou suck \n";
	}
	
	//edit entry (PLEASE TEST THIS FUNCTION ASAP).
	*/
	

	return 0;
}
