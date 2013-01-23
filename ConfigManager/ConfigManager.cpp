#include "ConfigManager.h"
#include "ConfigStorageManager.h"

#include <iostream>

using ConfigSystem::ConfigStorageManager;


namespace ConfigSystem
{
    
    ConfigManager::ConfigManager(std::string configName)
    {
    	CONFIGSYS_DEBUG_CALLS;

        // set initial values
        _configStore    = NULL;
        _currConfigTree = NULL;

        _configStore = new ConfigStorageManager();
    	loadConfiguration(configName);
    }

    ConfigManager::~ConfigManager()
    {
        CONFIGSYS_DEBUG_CALLS;

        delete _configStore     ; _configStore     = NULL;
        delete _currConfigTree  ; _currConfigTree  = NULL;
    }
    

    bool ConfigManager::loadConfiguration(std::string configName)
    {
        bool loaded = _configStore->loadConfig(_currConfigTree, configName);

        // If loading failed, output an error message
        if(!loaded)
        {
            std::cout << "ConfigManager::ConfigManager(std::string): "
                      << "Could not load configuration '"
                      << configName << "'."
                      << std::endl;
        }

        return loaded;
    }

    bool ConfigManager::saveConfiguration(std::string configName)
    {
        bool saved = _configStore->saveConfig(_currConfigTree, configName);

        // If saving failed, output an error message
        if(!saved)
        {
            std::cout << "ConfigManager::ConfigManager(std::string): "
                      << "Could not save configuration '"
                      << configName << "'."
                      << std::endl;
        }

        return saved;
    }
    
    
    // /*! @brief Reads an int     from the given path in the config system. */    
    // bool ConfigManager::readIntValue   (const std::string &paramPath,
    //                                     const std::string &paramName,
    //                                     int &data)
    // {
    //     CONFIGSYS_DEBUG_CALLS;

    //     ConfigParameter cp(vt_none);
    //     if(!_currConfigTree->getParam(paramPath, paramName, cp)) return false;
    //     return cp.getValue(data);
    // }

    /*! @brief Reads a  long    from the given path in the config system. */
    bool ConfigManager::readLongValue   (const std::string &paramPath,
                                         const std::string &paramName,
                                         long &data)
    {
        CONFIGSYS_DEBUG_CALLS;

        ConfigParameter cp(vt_none);
        if(!_currConfigTree->getParam(paramPath, paramName, cp)) return false;
        return cp.getValue(data);
    }
    // /*! @brief Reads a  float   from the given path in the config system. */
    // bool ConfigManager::readFloatParam  (const std::string &paramPath,
    //                                      const std::string &paramName,
    //                                      float  &data) // throw(ConfigException)
    // {
    //     CONFIGSYS_DEBUG_CALLS;

    //     ConfigParameter cp(vt_none);
    //     if(!_currConfigTree->getParam(paramPath, paramName, cp)) return false;
    //     return cp.getValue(data);
    // }

 //    /*! @brief Reads a  double  from the given path in the config system. */
    bool ConfigManager::readDoubleValue (const std::string &paramPath,
                                         const std::string &paramName,
                                         double &data)
    {
        CONFIGSYS_DEBUG_CALLS;

        ConfigParameter cp(vt_none);
        if(!_currConfigTree->getParam(paramPath, paramName, cp)) return false;
        return cp.getValue(data);
    }

    /*! @brief Reads a  string  from the given path in the config system. */
    bool ConfigManager::readStringValue (const std::string &paramPath,
                                         const std::string &paramName,
                                         std::string &data)
    {
        CONFIGSYS_DEBUG_CALLS;

        ConfigParameter cp(vt_none);
        if(!_currConfigTree->getParam(paramPath, paramName, cp)) return false;
        return cp.getValue(data);
    }


    // /*! @brief Stores the given int     data value into the config system at the given path. */
    // bool ConfigManager::storeIntValue    (const std::string &paramPath,
    //                                       const std::string &paramName,
    //                                       long data)
    // {
    //     CONFIGSYS_DEBUG_CALLS;
    //     //! Get the relevant parameter from the ConfigTree
    //     ConfigParameter cp(vt_none);
    //     if(!_currConfigTree->getParam(paramPath, paramName, cp)) return false;
    //     cp.setValue(data); //!< Set the new value
    //     //! Store the modified parameter back into the tree
    //     return _currConfigTree->storeParam(paramPath, paramName, cp);
    // }

    /*! @brief Stores the given long  data value into the config system at the given path. */
    bool ConfigManager::storeLongValue   (const std::string &paramPath,
                                          const std::string &paramName,
                                          long data)
    {
        CONFIGSYS_DEBUG_CALLS;
        //! Get the relevant parameter from the ConfigTree
        ConfigParameter cp(vt_none);
        if(!_currConfigTree->getParam(paramPath, paramName, cp)) return false;
        cp.setValue(data); //!< Set the new value
        //! Store the modified parameter back into the tree
        return _currConfigTree->storeParam(paramPath, paramName, cp);
    }

   //  /*! @brief Stores the given float   data value into the config system at the given path. */
   //  bool ConfigManager::storeFloatParam  (const std::string &paramPath,
   //                                        const std::string &paramName,
   //                                        float  data) // throw(ConfigException)
   // {
   //      CONFIGSYS_DEBUG_CALLS;
   //      //! Get the relevant parameter from the ConfigTree
   //      ConfigParameter cp(vt_none);
   //      if(!_currConfigTree->getParam(paramPath, paramName, cp)) return false;
   //      cp.setValue(data); //!< Set the new value
   //      //! Store the modified parameter back into the tree
   //      return _currConfigTree->storeParam(paramPath, paramName, cp);
   //  }

    /*! @brief Stores the given double  data value into the config system at the given path. */
    bool ConfigManager::storeDoubleValue (const string &paramPath,
                                          const string &paramName,
                                          double data)
    {
        CONFIGSYS_DEBUG_CALLS;
        //! Get the relevant parameter from the ConfigTree
        ConfigParameter cp(vt_none);
        if(!_currConfigTree->getParam(paramPath, paramName, cp)) return false;
        cp.setValue(data); //!< Set the new value
        //! Store the modified parameter back into the tree
        return _currConfigTree->storeParam(paramPath, paramName, cp);
    }
    
    /*! @brief Stores the given string  data value into the config system at the given path. */
    bool ConfigManager::storeStringValue (const string &paramPath,
                                          const string &paramName,
                                          std::string data)
    {
        CONFIGSYS_DEBUG_CALLS;
        //! Get the relevant parameter from the ConfigTree
        ConfigParameter cp(vt_none);
        if(!_currConfigTree->getParam(paramPath, paramName, cp)) return false;
        cp.setValue(data); //!< Set the new value
        //! Store the modified parameter back into the tree
        return _currConfigTree->storeParam(paramPath, paramName, cp);
    }

}
