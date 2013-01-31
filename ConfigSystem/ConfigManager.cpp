#include "ConfigManager.h"
#include "ConfigStorageManager.h"
#include "Configurable.h"

#include <iostream>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/foreach.hpp>
#include <vector>

using ConfigSystem::ConfigStorageManager;


namespace ConfigSystem
{
    
    ConfigManager::ConfigManager(std::string configName)
    {
    	CONFIGSYS_DEBUG_CALLS;

        // set initial values
        _configStore    = NULL;
        _currConfigTree = NULL;
        _configObjects  = std::vector<Configurable*>();

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
        else
        {
            // Send the new configuration to the configObjects
            // #warning Should occur within updateConfiguration()
            reconfigureConfigObjects();
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


    void ConfigManager::updateConfiguration()
    {
        // Update all ConfigObjects whose configurations have been outdated.
        updateConfigObjects();
    }


    bool ConfigManager::setConfigObjects(std::vector<Configurable*> configObjects)
    {
        _configObjects = configObjects;
        reconfigureConfigObjects();
        return true;
    }

    bool ConfigManager::addConfigObject(Configurable* configObject)
    {
        if(configObject == NULL) return false;
        _configObjects.push_back(configObject);
        configObject->loadConfig();
        configObject->setConfigAsRecent();
        return true;
    }
    
    void ConfigManager::reconfigureConfigObjects()
    {
        BOOST_FOREACH(Configurable* c, _configObjects)
        {
            if(c == NULL) continue;
            c->loadConfig();
            c->setConfigAsRecent();
        }
    }

    // void ConfigManager::updateConfigObjects(
    //     const std::string &paramPath,
    //     const std::string &paramName
    //     )
    void ConfigManager::updateConfigObjects()
    {
        // Should use some clever data structure to speed this up.
        // (such a data structure would be initialised within 
        // ConfigManager::setConfigObjects(...))
        BOOST_FOREACH(Configurable* c, _configObjects)
        {
            if(c == NULL) continue;
            // if(boost::starts_with(paramPath, c->_configBasePath))
            //     c->updateConfig(paramPath, paramName);

            if(c->isConfigOutdated())
            {
                c->updateConfig();
                c->setConfigAsRecent();
            }
        }
    }
    
    void ConfigManager::markConfigObjects(
            const std::string &paramPath,
            const std::string &paramName
            )
    {
        BOOST_FOREACH(Configurable* c, _configObjects)
        {
            if(c == NULL) continue;

            //! skip string comparison if already marked
            if(c->isConfigOutdated()) continue;

            //! Check whether the given path is on c's base path
            if(boost::starts_with(paramPath, c->getConfigBasePath()))
                c->setConfigAsOutdated();
        }
    }


    template<typename T>
    bool ConfigManager::readValue (
        const std::string &paramPath,
        const std::string &paramName,
        T &data
        )
    {
        CONFIGSYS_DEBUG_CALLS;
        ConfigParameter cp(vt_none);
        if(!_currConfigTree->getParam(paramPath, paramName, cp)) return false;
        return cp.getValue(data);
    }


    // New interface (using explicit template instantiations).
    template bool ConfigManager::readValue<long> (
        const std::string &paramPath, const std::string &paramName,
        long &data
        );
    template bool ConfigManager::readValue<double> (
        const std::string &paramPath, const std::string &paramName,
        double &data
        );
    template bool ConfigManager::readValue<std::string> (
        const std::string &paramPath, const std::string &paramName,
        std::string &data
        );
    template bool ConfigManager::readValue<std::vector<long> > (
        const std::string &paramPath, const std::string &paramName,
        std::vector<long> &data
        );
    template bool ConfigManager::readValue<std::vector<std::vector<long> > > (
        const std::string &paramPath, const std::string &paramName,
        std::vector<std::vector<long> > &data
        );
    template bool ConfigManager::readValue<std::vector<std::vector<std::vector<long> > > > (
        const std::string &paramPath, const std::string &paramName,
        std::vector<std::vector<std::vector<long> > > &data
        );
    template bool ConfigManager::readValue<std::vector<double> > (
        const std::string &paramPath, const std::string &paramName,
        std::vector<double>  &data
        );
    template bool ConfigManager::readValue<std::vector<std::vector<double> > > (
        const std::string &paramPath, const std::string &paramName,
        std::vector<std::vector<double> >  &data
        );
    template bool ConfigManager::readValue<std::vector<std::vector<std::vector<double> > > > (
        const std::string &paramPath, const std::string &paramName,
        std::vector<std::vector<std::vector<double> > > &data
        );

    // Old (explicitly named) interface.
    // This should probably be removed soon?
    bool ConfigManager::readLongValue   (
        const std::string &paramPath,  const std::string &paramName,
        long   &data
        ) { return readValue(paramPath, paramName, data); }
    bool ConfigManager::readDoubleValue (
        const std::string &paramPath,  const std::string &paramName,
        double &data
        ) { return readValue(paramPath, paramName, data); }
    bool ConfigManager::readStringValue (
        const std::string &paramPath,  const std::string &paramName,
        std::string &data
        ) { return readValue(paramPath, paramName, data); }
    bool ConfigManager::readLongVectorValue1D(
        const std::string &paramPath,  const std::string &paramName, 
        std::vector<long> &data
        ) { return readValue(paramPath, paramName, data); }
    bool ConfigManager::readLongVectorValue2D(
        const std::string &paramPath,  const std::string &paramName, 
        std::vector<std::vector<long> > &data
        ) { return readValue(paramPath, paramName, data); }
    bool ConfigManager::readLongVectorValue3D(
        const std::string &paramPath,  const std::string &paramName, 
        std::vector<std::vector<std::vector<long> > > &data
        ) { return readValue(paramPath, paramName, data); }
    bool ConfigManager::readDoubleVectorValue1D(
        const std::string &paramPath,  const std::string &paramName, 
        std::vector<double> &data
        ) { return readValue(paramPath, paramName, data); }
    bool ConfigManager::readDoubleVectorValue2D(
        const std::string &paramPath,  const std::string &paramName, 
        std::vector<std::vector<double> > &data
        ) { return readValue(paramPath, paramName, data); }
    bool ConfigManager::readDoubleVectorValue3D(
        const std::string &paramPath,  const std::string &paramName, 
        std::vector<std::vector<std::vector<double> > > &data
        ) { return readValue(paramPath, paramName, data); }




    template<typename T>
    bool ConfigManager::storeValue(
       const std::string &paramPath,
       const std::string &paramName,
       T data)
    {
        CONFIGSYS_DEBUG_CALLS;
        //! Get the relevant parameter from the ConfigTree
        ConfigParameter cp(vt_none);
        if(!_currConfigTree->getParam(paramPath, paramName, cp)) return false;
        
        //! Set the new value
        if(!cp.setValue(data)) return false; 
        
        //! Store the modified parameter back into the tree
        if(!_currConfigTree->storeParam(paramPath, paramName, cp)) return false;
        
        //! Request update of configObjects that depend on this parameter
        markConfigObjects(paramPath, paramName);

        return true;
    }
    
    // New interface (using explicit template instantiations).
    template bool ConfigManager::storeValue<long> (
        const std::string &paramPath, const std::string &paramName,
        long data
        );
    template bool ConfigManager::storeValue<double> (
        const std::string &paramPath, const std::string &paramName,
        double data
        );
    template bool ConfigManager::storeValue<std::string> (
        const std::string &paramPath, const std::string &paramName,
        std::string data
        );
    template bool ConfigManager::storeValue<std::vector<long> > (
        const std::string &paramPath, const std::string &paramName,
        std::vector<long> data
        );
    template bool ConfigManager::storeValue<std::vector<std::vector<long> > > (
        const std::string &paramPath, const std::string &paramName,
        std::vector<std::vector<long> > data
        );
    template bool ConfigManager::storeValue<std::vector<std::vector<std::vector<long> > > > (
        const std::string &paramPath, const std::string &paramName,
        std::vector<std::vector<std::vector<long> > > data
        );
    template bool ConfigManager::storeValue<std::vector<double> > (
        const std::string &paramPath, const std::string &paramName,
        std::vector<double>  data
        );
    template bool ConfigManager::storeValue<std::vector<std::vector<double> > > (
        const std::string &paramPath, const std::string &paramName,
        std::vector<std::vector<double> >  data
        );
    template bool ConfigManager::storeValue<std::vector<std::vector<std::vector<double> > > > (
        const std::string &paramPath, const std::string &paramName,
        std::vector<std::vector<std::vector<double> > > data
        );

    // Old (explicitly named) interface.
    // This should probably be removed soon?
    bool ConfigManager::storeLongValue   (
        const std::string &paramPath,  const std::string &paramName,
        long   data
        ) { return storeValue(paramPath, paramName, data); }
    bool ConfigManager::storeDoubleValue (
        const std::string &paramPath,  const std::string &paramName,
        double data
        ) { return storeValue(paramPath, paramName, data); }
    bool ConfigManager::storeStringValue (
        const std::string &paramPath,  const std::string &paramName,
        std::string data
        ) { return storeValue(paramPath, paramName, data); }
    bool ConfigManager::storeLongVectorValue1D(
        const std::string &paramPath,  const std::string &paramName, 
        std::vector<long> data
        ) { return storeValue(paramPath, paramName, data); }
    bool ConfigManager::storeLongVectorValue2D(
        const std::string &paramPath,  const std::string &paramName, 
        std::vector<std::vector<long> > data
        ) { return storeValue(paramPath, paramName, data); }
    bool ConfigManager::storeLongVectorValue3D(
        const std::string &paramPath,  const std::string &paramName, 
        std::vector<std::vector<std::vector<long> > > data
        ) { return storeValue(paramPath, paramName, data); }
    bool ConfigManager::storeDoubleVectorValue1D(
        const std::string &paramPath,  const std::string &paramName, 
        std::vector<double> data
        ) { return storeValue(paramPath, paramName, data); }
    bool ConfigManager::storeDoubleVectorValue2D(
        const std::string &paramPath,  const std::string &paramName, 
        std::vector<std::vector<double> > data
        ) { return storeValue(paramPath, paramName, data); }
    bool ConfigManager::storeDoubleVectorValue3D(
        const std::string &paramPath,  const std::string &paramName, 
        std::vector<std::vector<std::vector<double> > > data
        ) { return storeValue(paramPath, paramName, data); }




    bool ConfigManager::readDoubleRange(const string &paramPath, 
                                        const string &paramName, 
                            ConfigRange<double> &range)
    {
        CONFIGSYS_DEBUG_CALLS;

        ConfigParameter cp(vt_none);
        if(!_currConfigTree->getParam(paramPath, paramName, cp)) return false;
        return cp.getRange(range);
    }
    bool ConfigManager::readLongRange  (const string &paramPath, 
                                        const string &paramName, 
                                        ConfigRange<long> &range)
    {
        CONFIGSYS_DEBUG_CALLS;

        ConfigParameter cp(vt_none);
        if(!_currConfigTree->getParam(paramPath, paramName, cp)) return false;
        return cp.getRange(range);
    }

    bool ConfigManager::storeDoubleRange(const string &paramPath, 
                                        const string &paramName, 
                                        ConfigRange<double> &range)
    {
        CONFIGSYS_DEBUG_CALLS;
        //! Get the relevant parameter from the ConfigTree
        ConfigParameter cp(vt_none);
        if(!_currConfigTree->getParam(paramPath, paramName, cp)) return false;
        if(!cp.setRange(range)) return false; //!< Set the new value
        //! Store the modified parameter back into the tree
        return _currConfigTree->storeParam(paramPath, paramName, cp);
    }
    bool ConfigManager::storeLongRange (const string &paramPath, 
                                        const string &paramName, 
                                        ConfigRange<long> &range)
    {
        CONFIGSYS_DEBUG_CALLS;
        //! Get the relevant parameter from the ConfigTree
        ConfigParameter cp(vt_none);
        if(!_currConfigTree->getParam(paramPath, paramName, cp)) return false;
        if(!cp.setRange(range)) return false; //!< Set the new value
        //! Store the modified parameter back into the tree
        return _currConfigTree->storeParam(paramPath, paramName, cp);
    }
}
