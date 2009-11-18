/* NAO.cpp

 @author Jason Kulk
*/

#include <albroker.h>
#include <alproxy.h>
#include <almemoryproxy.h>
#include <almemoryfastaccess.h>

#include <iostream>
#include <fstream>

#include "NAO.h"
#include "../../config.h"

class NAO : public AL::ALModule
{
public:
    NAO(AL::ALPtr<AL::ALBroker> pBroker, const std::string& pName): ALModule(pBroker, pName)
    {
        setModuleDescription("A module that provides basic access to the NAOs sensors and actuators.");
        std::ofstream thelog;
        thelog.open("/var/log/jason.log");
        thelog << "NAO.cpp: NAO::NAO" << endl;
    }

    virtual ~NAO()
    {
    }
    
    void dataChanged(const std::string& pDataName, const ALValue& pValue, const std::string& pMessage)
    {
    }
    bool innerTest() 
    {
        return true;
    }
}; 

extern "C" int _createModule(AL::ALPtr<AL::ALBroker> pBroker)
{
    AL::ALModule::createModule<NAO>(pBroker, "NAO");
    cout << "NAO.cpp: _createModule" << endl;
    return 0;
}

extern "C" int _closeModule()
{
    return 0;
}





