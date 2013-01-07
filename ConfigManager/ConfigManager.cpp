#include "ConfigManager.h"
#include "ConfigStorageManager.h"

#include <iostream>
#include <boost/lexical_cast.hpp>

using CONFIGURATION::ConfigStorageManager;

/*main(int argc, const char** args)
{
    std::cout << "Begin" << std::endl;
    
    ptree pt;
    
    read_json("output.json", pt);
    
    write_json("output.json", pt);
    
    std::cout << "End" << std::endl;
}*/


namespace CONFIGURATION
{
	//constructor
	ConfigManager::ConfigManager(std::string filename_arr[])
	{
		bool stuff;
		stuff = storageManager.fileReadAll(filename_arr);
	}
	
	//destructor
	ConfigManager::~ConfigManager()
	{
		
	}
	
	//DEBUG
	//prints entire tree to file
	void ConfigManager::printAll()
	{
		std::string filename[ARR_SIZE];
		filename[0] = "WRITE_ConfigurationFiles/testconfig_write.json";
		filename[1] = "sdfs";
		filename[2] = "sdfs";
		filename[3] = "sdfs";
		filename[4] = "sdfs";
		filename[5] = "sdfs";
		
		storageManager.fileWriteAll(true, filename);
	}
	
	
	
	

	bool ConfigManager::readIntParam    (const std::string paramPath, int    &data) // throw(ConfigException);
	{
		// Get the parameter information from the pTree
		parameters<std::string> pStruct = storageManager.accessEntry(paramPath);
		int result;
		
		// Aliases for parameter strings
		std::string &value = pStruct.value;
		std::string &type  = pStruct.type;
		
		
		if(type.compare("int") != 0)
		{
			std::cout << "type not int" << "\n";
			return false; // type mismatch error. (throw ConfigTypeMismatchException?)
		}

		try
		{
			result = boost::lexical_cast<int>(value); // perform conversion from string
		}
		catch (const boost::bad_lexical_cast &blcExc) // conversion failed
		{
			// throw error?
			//set default value
			result = 0;
		}

		data = result; // assign to &data to output
		return true; // return success
	}

	bool ConfigManager::readLongParam   (const std::string paramPath, long   &data) // throw(ConfigException);
	{
		// Get the parameter information from the pTree
		parameters<std::string> pStruct = storageManager.accessEntry(paramPath);
		
		// Aliases for parameter strings
		std::string &value = pStruct.value;
		std::string &type  = pStruct.type;

		if(type.compare("long") != 0)
			return false; // type mismatch error. (throw ConfigTypeMismatchException?)

		long result;

		try
		{
			result = boost::lexical_cast<long>(value); // perform conversion from string
		}
		catch (const boost::bad_lexical_cast &blcExc) // conversion failed
		{
			// throw error?
			//set default value
			result = 0;
		}

		data = result; // assign to &data to output
		return true; // return success
	}

	bool ConfigManager::readFloatParam  (const std::string paramPath, float  &data) // throw(ConfigException);
	{
		// Get the parameter information from the pTree
		parameters<std::string> pStruct = storageManager.accessEntry(paramPath);
		
		// Aliases for parameter strings
		std::string &value = pStruct.value;
		std::string &type  = pStruct.type;

		if(type.compare("float") != 0)
			return false; // type mismatch error. (throw ConfigTypeMismatchException?)

		float result;

		try
		{
			result = boost::lexical_cast<float>(value); // perform conversion from string
		}
		catch (const boost::bad_lexical_cast &blcExc) // conversion failed
		{
			// throw error?
			//set default value
			result = 0;
		}

		data = result; // assign to &data to output
		return true; // return success
	}

	bool ConfigManager::readDoubleParam (const std::string paramPath, double &data) // throw(ConfigException);
	{
		// Get the parameter information from the pTree
		parameters<std::string> pStruct = storageManager.accessEntry(paramPath);
		
		// Aliases for parameter strings
		std::string &value = pStruct.value;
		std::string &type  = pStruct.type;

		if(type.compare("double") != 0)
			return false; // type mismatch error. (throw ConfigTypeMismatchException?)

		double result;

		try
		{
			result = boost::lexical_cast<double>(value); // perform conversion from string
		}
		catch (const boost::bad_lexical_cast &blcExc) // conversion failed
		{
			// throw error?
			//set default value
			result = 0;
		}

		data = result; // assign to &data to output
		return true; // return success
	}

	bool ConfigManager::readStringParam (const std::string paramPath, std::string &data) // throw(ConfigException);
	{
		// Get the parameter information from the pTree
		parameters<std::string> pStruct = storageManager.accessEntry(paramPath);
		
		// Aliases for parameter strings
		std::string &value = pStruct.value;
		std::string &type  = pStruct.type;

		if(type.compare("string") != 0)
			return false; // type mismatch error. (throw ConfigTypeMismatchException?)

		std::string result;

		result = value;
		// try
		// {
		// 	result = boost::lexical_cast<string>(value); // perform conversion from string
		// }
		// catch (const boost::bad_lexical_cast &blcExc) // conversion failed
		// {
		// 	// throw error?
		// 	//set default value
		// 	result = 0;
		// }

		data = result; // assign to &data to output
		return true; // return success
	}




	bool ConfigManager::storeIntParam    (const std::string paramPath, int    data) // throw(ConfigException)
	{
		//May need to retrieve rules such as "update_paths", etc to rewrite into struct object? Maybe a 
   		//better way? :S
		parameters<std::string> storing_data; 
		// convert data to a string
		std::string dataStr;
		
		try
		{
			dataStr = boost::lexical_cast<std::string>(data);
		}
		catch (const boost::bad_lexical_cast &blcExc) // conversion to string failed
		{
			// throw an error?
			return false;
		}
		
		storing_data.value = dataStr;
		storing_data.type = "int";
		
		bool success = storageManager.editEntry(paramPath, storing_data);
		
		return success;
	}

    bool ConfigManager::storeLongParam   (const std::string paramPath, long   data) // throw(ConfigException)
    {
   		//May need to retrieve rules such as "update_paths", etc to rewrite into struct object? Maybe a 
   		//better way? :S
		parameters<std::string> storing_data; 
    	// convert data to a string
    	std::string dataStr;
    	try
    	{
    		dataStr = boost::lexical_cast<std::string>(data);
    	}
    	catch (const boost::bad_lexical_cast &blcExc) // conversion to string failed
    	{
    		// throw an error?
    		return false;
    	}
    	
    	storing_data.value = dataStr;
    	storing_data.type = "long";

    	bool success = storageManager.editEntry(paramPath, storing_data);
    	
    	return success;
    }

    bool ConfigManager::storeFloatParam  (const std::string paramPath, float  data) // throw(ConfigException)
    {
    	//May need to retrieve rules such as "update_paths", etc to rewrite into struct object? Maybe a 
   		//better way? :S
		parameters<std::string> storing_data; 
    	// convert data to a string
    	std::string dataStr;
    	try
    	{
    		dataStr = boost::lexical_cast<std::string>(data);
    	}
    	catch (const boost::bad_lexical_cast &blcExc) // conversion to string failed
    	{
    		// throw an error?
    		return false;
    	}
    	
    	storing_data.value = dataStr;
    	storing_data.type = "float";

    	bool success = storageManager.editEntry(paramPath, storing_data);
    	
    	return success;
    }

    bool ConfigManager::storeDoubleParam (const std::string paramPath, double data) // throw(ConfigException)
    {
    	//May need to retrieve rules such as "update_paths", etc to rewrite into struct object? Maybe a 
   		//better way? :S
		parameters<std::string> storing_data; 
    	// convert data to a string
    	std::string dataStr;
    	try
    	{
    		dataStr = boost::lexical_cast<std::string>(data);
    	}
    	catch (const boost::bad_lexical_cast &blcExc) // conversion to string failed
    	{
    		// throw an error?
    		return false;
    	}
    	
    	storing_data.value = dataStr;
    	storing_data.type = "double";

    	bool success = storageManager.editEntry(paramPath, storing_data);
    	
    	return success;
    }

    bool ConfigManager::storeStringParam (const std::string paramPath, std::string data) // throw(ConfigException)
    {
    	//May need to retrieve rules such as "update_paths", etc to rewrite into struct object? Maybe a 
   		//better way? :S
		parameters<std::string> storing_data; 
    	// convert data to a string
    	std::string dataStr;
    	try
    	{
    		dataStr = boost::lexical_cast<std::string>(data);
    	}
    	catch (const boost::bad_lexical_cast &blcExc) // conversion to string failed
    	{
    		// throw an error?
    		return false;
    	}
    	
    	storing_data.value = dataStr;
    	storing_data.type = "string";

    	bool success = storageManager.editEntry(paramPath, storing_data);
    	
    	return success;
    }

}
