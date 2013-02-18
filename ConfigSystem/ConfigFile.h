/*! @file 	ConfigFile.h
    @brief 	This is the header file for the ConfigFile class.
    
    @class 	ConfigFile
    @brief 	Stores a single base path to a config parameter. Used as a list in the CSM of filenames to read
    		and write.
           
    The Configurable class stores a base path to the parameter to keep track of, and a modified flag. 
    Whenever a parameter's value is modified (in storeValue or createParam) 'markConfigObjects()' is
    called, which marks all the 'Configurable' objects (in ConfigManager's _configObjects list) as 
    needing an update. 
    On every iteration of the see-think thread, 'updateConfiguration()' is called, which calls each of 
    the marked Configurables' 'Configurabe::updateConfig()' methods.
    
	NOTE: Users of the config system have to imlement the 'Configurable::updateConfig()' method 
	themselves.
    
    @author Mitchell Metcalfe, Sophie Calland
    
  Copyright (c) 2012 Mitchell Metcalfe
    
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


class ConfigFile
{
	public:
		ConfigFile(std::string base_path);
		~ConfigFile();
		
		bool getBasePath(std::string &base_path);
		bool setBasePath(std::string base_path);
		
	private:
		std::string _base_path;
}
