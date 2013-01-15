#include "ConfigManager.h"
#include "ConfigStorageManager.h"

#include <iostream>

using ConfigSystem::ConfigStorageManager;

/*main(int argc, const char** args)
{
    std::cout << "Begin" << std::endl;
    
    ptree pt;
    
    read_json("output.json", pt);
    
    write_json("output.json", pt);
    
    std::cout << "End" << std::endl;
}*/


namespace ConfigSystem
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
	
	
 //    /*! @brief Reads an int     from the given path in the config system. */    
 //    bool ConfigManager::readIntParam    (const string &paramPath,
 //                                         const string &paramName,
 //                                         int    &data) // throw(ConfigException)
 //    {
 //        return readParam<int    >(paramPath, paramName, data, "int"    );
 //    }

 //    /*! @brief Reads a  long    from the given path in the config system. */
 //    bool ConfigManager::readLongParam   (const string &paramPath,
 //                                         const string &paramName,
 //                                         long   &data) // throw(ConfigException)
 //    {
 //        return readParam<long   >(paramPath, paramName, data, "long"   );
 //    }

 //    /*! @brief Reads a  float   from the given path in the config system. */
 //    bool ConfigManager::readFloatParam  (const string &paramPath,
 //                                         const string &paramName,
 //                                         float  &data) // throw(ConfigException)
 //    {
 //        return readParam<float  >(paramPath, paramName, data, "float"  );
 //    }

 //    /*! @brief Reads a  double  from the given path in the config system. */
 //    bool ConfigManager::readDoubleParam (const string &paramPath,
 //                                         const string &paramName,
 //                                         double &data) // throw(ConfigException)
 //    {
 //        return readParam<double >(paramPath, paramName, data, "double" );
 //    }

 //    /*! @brief Reads a  string  from the given path in the config system. */
 //    bool ConfigManager::readStringParam (const string &paramPath,
 //                                         const string &paramName,
 //                                         string &data) // throw(ConfigException)
 //    {
 //        return readParam<string >(paramPath, paramName, data, "string" );
 //    }


 //    ! @brief Stores the given int     data value into the config system at the given path. 
 //    bool ConfigManager::storeIntParam    (const string &paramPath,
 //                                          const string &paramName,
 //                                          int    data) // throw(ConfigException)
 //    {
 //        return storeParam<int   >(paramPath, paramName, data, "int"    );
 //    }

 //    /*! @brief Stores the given long    data value into the config system at the given path. */
 //    bool ConfigManager::storeLongParam   (const string &paramPath,
 //                                          const string &paramName,
 //                                          long   data) // throw(ConfigException)
 //    {
 //        return storeParam<long  >(paramPath, paramName, data, "long"   );
 //    }

 //    /*! @brief Stores the given float   data value into the config system at the given path. */
 //    bool ConfigManager::storeFloatParam  (const string &paramPath,
 //                                          const string &paramName,
 //                                          float  data) // throw(ConfigException)
 //    {
 //        return storeParam<float >(paramPath, paramName, data, "float"  );
 //    }

 //    /*! @brief Stores the given double  data value into the config system at the given path. */
 //    bool ConfigManager::storeDoubleParam (const string &paramPath,
 //                                          const string &paramName,
 //                                          double data) // throw(ConfigException)
 //    {
 //        return storeParam<double>(paramPath, paramName, data, "double" );
 //    }

 //    /*! @brief Stores the given string  data value into the config system at the given path. */
 //    bool ConfigManager::storeStringParam (const string &paramPath,
 //                                          const string &paramName,
 //                                          string data) // throw(ConfigException)
 //    {
 //        return storeParam<string>(paramPath, paramName, data, "string" );
 //    }

}
