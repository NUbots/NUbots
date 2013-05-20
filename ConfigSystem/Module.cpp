/*! @file Module.cpp
    @brief 
    
    @author Mitchell Metcalfe
*/

#include "Module.h"
#include "Testing_MM_Utils.h"
#include "ConfigManager.h"


#include <boost/foreach.hpp>
#include <vector>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>


// #error Define CONFIGSYSTEM_TEST_ROBOT when compiling on the robot (else code won't compile). 
// Comment out the define otherwise (else test results will be meaningless).
#warning Ensure that CONFIGSYSTEM_TEST_ROBOT is defined correctly.
#define CONFIGSYSTEM_TEST_ROBOT

#ifdef CONFIGSYSTEM_TEST_ROBOT
    #include "Infrastructure/NUBlackboard.h"
#endif

using namespace ConfigSystem;


Module::Module()
{
    _firstLoad = true;

    _shouldUpdate = false;
    _updateCalled = false;

    name_3dv_d = "name_3dv_d";
    name_3dv_l = "name_3dv_l";
    name_2dv_d = "name_2dv_d";
    name_2dv_l = "name_2dv_l";
    name_1dv_d = "name_1dv_d";
    name_1dv_l = "name_1dv_l";
    name_d     = "name_d    ";
    name_l     = "name_l    ";
    name_s     = "name_s    ";
    val_changed_3dv_d = false;
    val_changed_3dv_l = false;
    val_changed_2dv_d = false;
    val_changed_2dv_l = false;
    val_changed_1dv_d = false;
    val_changed_1dv_l = false;
    val_changed_d     = false;
    val_changed_l     = false;
    val_changed_s     = false;
    create3DTestVector(val_3dv_d);
    create3DTestVector(val_3dv_l);
    create2DTestVector(val_2dv_d);
    create2DTestVector(val_2dv_l);
    create1DTestVector(val_1dv_d);
    create1DTestVector(val_1dv_l);
    createTestValue   (val_d    );
    createTestValue   (val_l    );
    val_s = makeRandomName()     ;
    val_3dv_d_new = val_3dv_d;
    val_3dv_l_new = val_3dv_l;
    val_2dv_d_new = val_2dv_d;
    val_2dv_l_new = val_2dv_l;
    val_1dv_d_new = val_1dv_d;
    val_1dv_l_new = val_1dv_l;
    val_d_new     = val_d    ;
    val_l_new     = val_l    ;
    val_s_new     = val_s    ;
}


// Create params
// NOTE: 'loadConfig()' was never intended to contain calls to 
// 'CreateParam(...)' for all of its parameters.
// It was just convenient to write tests.
void Module::loadConfig()
{
    #ifdef CONFIGSYSTEM_TEST_ROBOT
        ConfigManager* config = Blackboard->Config;
    #endif

    if(_firstLoad)
    {
        _firstLoad = false;

        create3DTestVector(val_3dv_d);
        create3DTestVector(val_3dv_l);
        create2DTestVector(val_2dv_d);
        create2DTestVector(val_2dv_l);
        create1DTestVector(val_1dv_d);
        create1DTestVector(val_1dv_l);
        createTestValue   (val_d);
        createTestValue   (val_l);
        val_s = makeRandomName();
        config->CreateParam(_configBasePath, name_3dv_d, val_3dv_d);
        config->CreateParam(_configBasePath, name_3dv_l, val_3dv_l);
        config->CreateParam(_configBasePath, name_2dv_d, val_2dv_d);
        config->CreateParam(_configBasePath, name_2dv_l, val_2dv_l);
        config->CreateParam(_configBasePath, name_1dv_d, val_1dv_d);
        config->CreateParam(_configBasePath, name_1dv_l, val_1dv_l);
        config->CreateParam(_configBasePath, name_d    , val_d    );
        config->CreateParam(_configBasePath, name_l    , val_l    );
        config->CreateParam(_configBasePath, name_s    , val_s    );
    }
    else
    {
        config->ReadValue(_configBasePath, name_3dv_d, &val_3dv_d);
        config->ReadValue(_configBasePath, name_3dv_l, &val_3dv_l);
        config->ReadValue(_configBasePath, name_2dv_d, &val_2dv_d);
        config->ReadValue(_configBasePath, name_2dv_l, &val_2dv_l);
        config->ReadValue(_configBasePath, name_1dv_d, &val_1dv_d);
        config->ReadValue(_configBasePath, name_1dv_l, &val_1dv_l);
        config->ReadValue(_configBasePath, name_d    , &val_d    );
        config->ReadValue(_configBasePath, name_l    , &val_l    );
        config->ReadValue(_configBasePath, name_s    , &val_s    );
    }

    _shouldUpdate = false;
    _updateCalled = false;

    val_changed_3dv_d = false;
    val_changed_3dv_l = false;
    val_changed_2dv_d = false;
    val_changed_2dv_l = false;
    val_changed_1dv_d = false;
    val_changed_1dv_l = false;
    val_changed_d     = false;
    val_changed_l     = false;
    val_changed_s     = false;
    val_3dv_d_new = val_3dv_d;
    val_3dv_l_new = val_3dv_l;
    val_2dv_d_new = val_2dv_d;
    val_2dv_l_new = val_2dv_l;
    val_1dv_d_new = val_1dv_d;
    val_1dv_l_new = val_1dv_l;
    val_d_new     = val_d    ;
    val_l_new     = val_l    ;
    val_s_new     = val_s    ;
}

void Module::updateConfig()
{
    _updateCalled = true;

    #ifdef CONFIGSYSTEM_TEST_ROBOT
        ConfigManager* config = Blackboard->Config;
    #endif
    config->ReadValue(_configBasePath, name_3dv_d, &val_3dv_d);
    config->ReadValue(_configBasePath, name_3dv_l, &val_3dv_l);
    config->ReadValue(_configBasePath, name_2dv_d, &val_2dv_d);
    config->ReadValue(_configBasePath, name_2dv_l, &val_2dv_l);
    config->ReadValue(_configBasePath, name_1dv_d, &val_1dv_d);
    config->ReadValue(_configBasePath, name_1dv_l, &val_1dv_l);
    config->ReadValue(_configBasePath, name_d    , &val_d    );
    config->ReadValue(_configBasePath, name_l    , &val_l    );
    config->ReadValue(_configBasePath, name_s    , &val_s    );
}


bool Module::compare()
{
    bool result = true;
    
    // check all values
    bool res_3dv_d = compare3DVectors(val_3dv_d_new, val_3dv_d, false);
    bool res_3dv_l = compare3DVectors(val_3dv_l_new, val_3dv_l, false);
    bool res_2dv_d = compare2DVectors(val_2dv_d_new, val_2dv_d, false);
    bool res_2dv_l = compare2DVectors(val_2dv_l_new, val_2dv_l, false);
    bool res_1dv_d = compare1DVectors(val_1dv_d_new, val_1dv_d, false);
    bool res_1dv_l = compare1DVectors(val_1dv_l_new, val_1dv_l, false);
    bool res_d     = compareValues   (val_d_new, val_d    );
    bool res_l     = compareValues   (val_l_new, val_l    );
    // should string vals too 
    
    // if tests fail, recompare + print failures
    if(!res_3dv_d)
    {
        std::cout << std::endl << "val_3dv_d failed:" << std::endl;
        compare3DVectors(val_3dv_d_new, val_3dv_d); 
    }
    if(!res_3dv_l)
    {
        std::cout << std::endl << "val_3dv_l failed:" << std::endl;
        compare3DVectors(val_3dv_l_new, val_3dv_l); 
    }
    if(!res_2dv_d)
    {
        std::cout << std::endl << "val_2dv_d failed:" << std::endl;
        compare2DVectors(val_2dv_d_new, val_2dv_d); 
    }
    if(!res_2dv_l)
    {
        std::cout << std::endl << "val_2dv_l failed:" << std::endl;
        compare2DVectors(val_2dv_l_new, val_2dv_l); 
    }
    if(!res_1dv_d)
    {
        std::cout << std::endl << "val_1dv_d failed:" << std::endl;
        compare1DVectors(val_1dv_d_new, val_1dv_d); 
    }
    if(!res_1dv_l)
    {
        std::cout << std::endl << "val_1dv_l failed:" << std::endl;
        compare1DVectors(val_1dv_l_new, val_1dv_l); 
    }
    if(!res_d    )
    {
        std::cout << std::endl << "val_d     failed:" << std::endl;
        compareValues   (val_d_new    , val_d    ); 
    }
    if(!res_l    )
    {
        std::cout << std::endl << "val_l     failed:" << std::endl;
        compareValues   (val_l_new    , val_l    ); 
    }

    // combine with result
    result &= res_3dv_d;
    result &= res_3dv_l;
    result &= res_2dv_d;
    result &= res_2dv_l;
    result &= res_1dv_d;
    result &= res_1dv_l;
    result &= res_d    ;
    result &= res_l    ;

    // either needed to update and did,
    // or didn't need to update and didn't
    bool correctUpdate = !(_shouldUpdate ^ _updateCalled);
    if(!correctUpdate)
    {
        std::cout 
            << __PRETTY_FUNCTION__ 
            << ": "
            << "Should " 
            << (_shouldUpdate? "" : "not ") 
            << "have called 'update()', "
            << "but 'update()' was " 
            << (_updateCalled? "" : "not ") 
            << "called."
            << std::endl;
    }
    result &= correctUpdate;
    _shouldUpdate = false;
    _updateCalled = false;

    if(!result)
    {
        std::cout << "    val_changed_3dv_d = " << ((val_changed_3dv_d)? "true" : "false") << std::endl;
        std::cout << "    val_changed_3dv_l = " << ((val_changed_3dv_l)? "true" : "false") << std::endl;
        std::cout << "    val_changed_2dv_d = " << ((val_changed_2dv_d)? "true" : "false") << std::endl;
        std::cout << "    val_changed_2dv_l = " << ((val_changed_2dv_l)? "true" : "false") << std::endl;
        std::cout << "    val_changed_1dv_d = " << ((val_changed_1dv_d)? "true" : "false") << std::endl;
        std::cout << "    val_changed_1dv_l = " << ((val_changed_1dv_l)? "true" : "false") << std::endl;
        std::cout << "    val_changed_d     = " << ((val_changed_d    )? "true" : "false") << std::endl;
        std::cout << "    val_changed_l     = " << ((val_changed_l    )? "true" : "false") << std::endl;
        std::cout << "    val_changed_s     = " << ((val_changed_s    )? "true" : "false") << std::endl;
    }

    // Reset change flags 
    val_changed_3dv_d = false;
    val_changed_3dv_l = false;
    val_changed_2dv_d = false;
    val_changed_2dv_l = false;
    val_changed_1dv_d = false;
    val_changed_1dv_l = false;
    val_changed_d     = false;
    val_changed_l     = false;
    val_changed_s     = false;

    return result;
}


// Note: this method doesn't guarentee that anything should change.
//       (this is deliberate - flagging of changes is done in the
//        methods for each parameter)
void Module::change()
{
    // Check that distBool doesn't always generate the same number,
    // + change it's range slightly if it does. 
    boost::uniform_int<> distBool(0, 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > randBool(seed,distBool);

    // choose random values to change
    if(randBool() != 0) change_3dv_d();
    if(randBool() != 0) change_3dv_l();
    if(randBool() != 0) change_2dv_d();
    if(randBool() != 0) change_2dv_l();
    if(randBool() != 0) change_1dv_d();
    if(randBool() != 0) change_1dv_l();
    if(randBool() != 0) change_d    ();
    if(randBool() != 0) change_l    ();
    if(randBool() != 0) change_s    ();
}
    
void Module::change_3dv_d()
{
    #ifdef CONFIGSYSTEM_TEST_ROBOT
        ConfigManager* config = Blackboard->Config;
    #endif
    create3DTestVector(val_3dv_d_new);
    config->SetValue(_configBasePath, name_3dv_d, val_3dv_d_new);
    val_changed_3dv_d = true;
    _shouldUpdate = true;
}
void Module::change_3dv_l()
{
    #ifdef CONFIGSYSTEM_TEST_ROBOT
        ConfigManager* config = Blackboard->Config;
    #endif
    create3DTestVector(val_3dv_l_new);
    config->SetValue(_configBasePath, name_3dv_l, val_3dv_l_new);
    val_changed_3dv_l = true;
    _shouldUpdate = true;
}
void Module::change_2dv_d()
{
    #ifdef CONFIGSYSTEM_TEST_ROBOT
        ConfigManager* config = Blackboard->Config;
    #endif
    create2DTestVector(val_2dv_d_new);
    config->SetValue(_configBasePath, name_2dv_d, val_2dv_d_new);
    val_changed_2dv_d = true;
    _shouldUpdate = true;
}
void Module::change_2dv_l()
{
    #ifdef CONFIGSYSTEM_TEST_ROBOT
        ConfigManager* config = Blackboard->Config;
    #endif
    create2DTestVector(val_2dv_l_new);
    config->SetValue(_configBasePath, name_2dv_l, val_2dv_l_new);
    val_changed_2dv_l = true;
    _shouldUpdate = true;
}
void Module::change_1dv_d()
{
    #ifdef CONFIGSYSTEM_TEST_ROBOT
        ConfigManager* config = Blackboard->Config;
    #endif
    create1DTestVector(val_1dv_d_new);
    config->SetValue(_configBasePath, name_1dv_d, val_1dv_d_new);
    val_changed_1dv_d = true;
    _shouldUpdate = true;
}
void Module::change_1dv_l()
{
    #ifdef CONFIGSYSTEM_TEST_ROBOT
        ConfigManager* config = Blackboard->Config;
    #endif
    create1DTestVector(val_1dv_l_new);
    config->SetValue(_configBasePath, name_1dv_l, val_1dv_l_new);
    val_changed_1dv_l = true;
    _shouldUpdate = true;
}
void Module::change_d    ()
{
    #ifdef CONFIGSYSTEM_TEST_ROBOT
        ConfigManager* config = Blackboard->Config;
    #endif
    createTestValue   (val_d_new    );
    config->SetValue(_configBasePath, name_d    , val_d_new    );
    val_changed_d     = true;
    _shouldUpdate = true;
}
void Module::change_l    ()
{
    #ifdef CONFIGSYSTEM_TEST_ROBOT
        ConfigManager* config = Blackboard->Config;
    #endif
    createTestValue   (val_l_new    );
    config->SetValue(_configBasePath, name_l    , val_l_new    );
    val_changed_l     = true;
    _shouldUpdate = true;
}
void Module::change_s    ()
{ 
    #ifdef CONFIGSYSTEM_TEST_ROBOT
        ConfigManager* config = Blackboard->Config;
    #endif
    val_s_new    = makeRandomName();
    config->SetValue(_configBasePath, name_s    , val_s_new    );
    val_changed_s     = true;
    _shouldUpdate = true;
}


bool Module::autoUpdateTest()
{
    static bool firstRun = true;
    static std::vector<Module> _modules;
    static int num_modules = 50;

    #ifdef CONFIGSYSTEM_TEST_ROBOT
        ConfigManager* config = Blackboard->Config;
    #endif
    
    bool result = true;

    // if this is the first call to autoUpdateTest()
    if(firstRun) // Note: segfaults if this doesn't run.
    {
        firstRun = false;

        // create Modules
        for(int i = 0; i < num_modules; i++)
        {
            Module m;

            Configurable* c = &m;
            c->setConfigBasePath(makeRandomName());

            _modules.push_back(m);
        }

        std::vector<Configurable*> cfObjs;
        // add modules to configObjects
        BOOST_FOREACH(Module &m, _modules)
        {
            cfObjs.push_back(&m);
        }
        config->SetConfigObjects(cfObjs);
    }
    else // on each call to autoUpdateTest() after the first:
    {

        BOOST_FOREACH(Module &m, _modules)
        {
            // Compare changes from auto-update
            result &= m.compare();
        }
        printTestResult("autoUpdateTest", result);

        // Make new changes
        boost::uniform_int<> distI(0, num_modules - 1);
        boost::variate_generator<boost::mt19937&, boost::uniform_int<> > randInt(seed,distI);
        
        int numChanges = randInt(); // choose the number of changes to make
        std::cout << "Changes = " << numChanges << std::endl;
        for(int i = 0; i < numChanges; i++)
        {
            int index = randInt(); // select which module to change
            _modules[index].change(); // randomly change the module
        }

        // Update the configuration
        // (called within See-Think Thread, unless testing off-robot)
        #ifndef CONFIGSYSTEM_TEST_ROBOT
            config->UpdateConfiguration(); 
        #endif
    }

    return result;
}
